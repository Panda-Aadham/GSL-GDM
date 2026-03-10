from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from olfaction_msgs.msg import Anemometer, GasSensor
from rclpy.node import Node
from std_msgs.msg import Header

from .gdm.common import DiscreteScalarMap, Observation
from .gdm.kdm import KDM_VW
from .map_io import load_occupancy_map


class KdmVwMapperNode(Node):
    def __init__(self):
        super().__init__("kdm_vw_mapper")

        self.declare_parameter("scenario", "House01")
        self.declare_parameter("simulation", "1,3-2,4_fast")
        self.declare_parameter("occupancy_yaml", "")
        self.declare_parameter("gas_topic", "PID/Sensor_reading")
        self.declare_parameter("wind_topic", "Anemometer/WindSensor_reading")
        self.declare_parameter("pose_topic", "ground_truth")
        self.declare_parameter("gas_estimate_topic", "KDM_VW/gas_estimate")
        self.declare_parameter("gas_uncertainty_topic", "KDM_VW/gas_uncertainty")
        self.declare_parameter("exploration_path_topic", "KDM_VW/exploration_path")
        self.declare_parameter("update_period", 1.0)
        self.declare_parameter("min_observation_spacing", 0.15)
        self.declare_parameter("min_observation_period", 0.25)
        self.declare_parameter("observation_check_period", 0.1)
        self.declare_parameter("max_measurement_age", 2.0)
        self.declare_parameter("output_dir", "results/KDM_VW")

        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value
        self.simulation = self.get_parameter("simulation").get_parameter_value().string_value
        self.occupancy_yaml = self.get_parameter("occupancy_yaml").get_parameter_value().string_value
        self.output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        if not self.occupancy_yaml:
            raise RuntimeError("The 'occupancy_yaml' parameter must point to a VGR occupancy yaml file.")

        self.map_metadata = load_occupancy_map(self.occupancy_yaml)
        self.domain_map = DiscreteScalarMap(
            dimensions=2,
            size=self.map_metadata.size,
            resolution=self.map_metadata.resolution,
            offset=self.map_metadata.offset,
        )
        self.mapper = KDM_VW(self.domain_map)
        self.free_mask = self.map_metadata.free_mask

        self.latest_pose = None
        self.latest_wind_xy = (0.0, 0.0)
        self.latest_gas_ppm = None
        self.latest_gas_time = None
        self.last_observation_position = None
        self.last_observation_time = None
        self.observations = []
        self.path_msg = NavPath()
        self.path_msg.header.frame_id = "map"
        self.has_new_observation = False

        self.min_observation_spacing = self.get_parameter("min_observation_spacing").value
        self.min_observation_period = self.get_parameter("min_observation_period").value
        self.max_measurement_age = self.get_parameter("max_measurement_age").value

        self.gas_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("gas_estimate_topic").get_parameter_value().string_value,
            10,
        )
        self.uncertainty_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("gas_uncertainty_topic").get_parameter_value().string_value,
            10,
        )
        self.path_pub = self.create_publisher(
            NavPath,
            self.get_parameter("exploration_path_topic").get_parameter_value().string_value,
            10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").get_parameter_value().string_value,
            self.pose_callback,
            10,
        )
        self.create_subscription(
            GasSensor,
            self.get_parameter("gas_topic").get_parameter_value().string_value,
            self.gas_callback,
            10,
        )
        self.create_subscription(
            Anemometer,
            self.get_parameter("wind_topic").get_parameter_value().string_value,
            self.wind_callback,
            10,
        )

        self.observation_timer = self.create_timer(
            float(self.get_parameter("observation_check_period").value),
            self.sample_observation,
        )
        self.timer = self.create_timer(float(self.get_parameter("update_period").value), self.update_map)

        self.get_logger().info(
            f"KDM+V/W mapper ready for {self.scenario}/{self.simulation} using map '{self.occupancy_yaml}'."
        )

    def pose_callback(self, msg):
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def wind_callback(self, msg):
        direction = float(msg.wind_direction)
        speed = float(msg.wind_speed)
        self.latest_wind_xy = (
            speed * np.cos(direction),
            speed * np.sin(direction),
        )

    def gas_callback(self, msg):
        self.latest_gas_ppm = self.ppm_from_gas_msg(msg)
        self.latest_gas_time = float(self.get_clock().now().nanoseconds) / 1e9

    def sample_observation(self):
        if self.latest_pose is None or self.latest_gas_ppm is None or self.latest_gas_time is None:
            return

        now_seconds = float(self.get_clock().now().nanoseconds) / 1e9
        if now_seconds - self.latest_gas_time > self.max_measurement_age:
            return

        if self.last_observation_position is not None:
            dx = self.latest_pose[0] - self.last_observation_position[0]
            dy = self.latest_pose[1] - self.last_observation_position[1]
            if np.hypot(dx, dy) < self.min_observation_spacing:
                if self.last_observation_time is not None and now_seconds - self.last_observation_time < self.min_observation_period:
                    return

        observation = Observation(
            position=self.latest_pose,
            gas=max(0.0, self.latest_gas_ppm),
            wind=self.latest_wind_xy,
            time=now_seconds,
            data_type="gas+wind",
        )
        self.mapper.addObservation(observation)
        self.observations.append(observation)
        self.last_observation_position = self.latest_pose
        self.last_observation_time = now_seconds
        self.has_new_observation = True
        self.append_path_pose()

        if len(self.observations) == 1 or len(self.observations) % 25 == 0:
            self.get_logger().info(
                f"Stored {len(self.observations)} observations. Latest gas={self.latest_gas_ppm:.3f} ppm wind=({self.latest_wind_xy[0]:.3f}, {self.latest_wind_xy[1]:.3f})."
            )

    def update_map(self):
        if not self.has_new_observation:
            return

        self.mapper.estimate()
        gas_matrix = self.mapper.getGasEstimate().toMatrix()
        uncertainty_matrix = self.mapper.getGasUncertainty().toMatrix()

        self.publish_grid(self.gas_pub, gas_matrix, "gas_estimate")
        self.publish_grid(self.uncertainty_pub, uncertainty_matrix, "gas_uncertainty")
        self.publish_path()
        self.save_outputs(gas_matrix, uncertainty_matrix)

        self.has_new_observation = False
        self.get_logger().info(f"Published KDM+V/W maps from {len(self.observations)} observations.")

    def publish_grid(self, publisher, matrix, kind):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = float(self.map_metadata.resolution)
        grid.info.width = self.map_metadata.width
        grid.info.height = self.map_metadata.height
        grid.info.origin.position.x = float(self.map_metadata.origin_x)
        grid.info.origin.position.y = float(self.map_metadata.origin_y)
        grid.info.origin.orientation.w = 1.0

        normalized = self.normalize_for_display(matrix)
        normalized[~self.free_mask] = -1
        grid.data = np.flipud(normalized).astype(np.int8).flatten().tolist()
        publisher.publish(grid)

    def append_path_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.latest_pose[0])
        pose.pose.position.y = float(self.latest_pose[1])
        pose.pose.orientation.w = 1.0
        self.path_msg.poses.append(pose)
        self.publish_path()

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def normalize_for_display(self, matrix):
        finite = np.isfinite(matrix)
        if not finite.any():
            return np.zeros(matrix.shape, dtype=int)

        working = np.array(matrix, dtype=float, copy=True)
        working[~finite] = 0.0
        maximum = float(np.max(working))
        if maximum <= 1e-9:
            return np.zeros(matrix.shape, dtype=int)

        return np.clip((working / maximum) * 100.0, 0.0, 100.0).astype(int)

    def save_outputs(self, gas_matrix, uncertainty_matrix):
        stem = f"{self.scenario}_{self.simulation}"
        np.savetxt(self.output_dir / f"{stem}_gas_estimate.csv", gas_matrix, delimiter=",")
        np.savetxt(self.output_dir / f"{stem}_gas_uncertainty.csv", uncertainty_matrix, delimiter=",")

        observations_path = self.output_dir / f"{stem}_observations.csv"
        with observations_path.open("w", encoding="utf-8") as handle:
            handle.write("x,y,gas_ppm,wind_x,wind_y,time\n")
            for observation in self.observations:
                handle.write(
                    f"{observation.position[0]},{observation.position[1]},{observation.gas},"
                    f"{observation.wind[0]},{observation.wind[1]},{observation.time}\n"
                )

    def ppm_from_gas_msg(self, msg):
        if msg.raw_units == GasSensor.UNITS_PPM:
            return float(msg.raw)

        if msg.raw_units == GasSensor.UNITS_OHM and msg.calib_a != 0.0 and msg.calib_b != 0.0:
            rs_r0 = msg.raw / 50000.0
            return float(np.power(rs_r0 / msg.calib_a, 1.0 / msg.calib_b))

        self.get_logger().warn(f"Unsupported gas units {msg.raw_units}, returning 0 ppm.")
        return 0.0


def main():
    rclpy.init()
    node = KdmVwMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
