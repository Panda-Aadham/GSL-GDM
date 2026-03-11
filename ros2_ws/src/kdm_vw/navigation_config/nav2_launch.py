import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_arguments():
    default_map_file = os.path.join(
        get_package_share_directory("vgr_dataset"),
        "scenarios",
        "House01",
        "occupancy.yaml",
    )

    return [
        DeclareLaunchArgument("namespace", default_value="PioneerP3DX"),
        DeclareLaunchArgument("scenario", default_value="House01"),
        DeclareLaunchArgument("map_file", default_value=default_map_file),
        DeclareLaunchArgument("robot_radius", default_value="0.05"),
        DeclareLaunchArgument("log_level", default_value="ERROR"),
    ]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    map_file = LaunchConfiguration("map_file").perform(context)

    configured_params = ParameterFile(
        LaunchConfiguration("nav_params_yaml").perform(context),
        allow_substs=True,
    )
    use_sim_time = True

    navigation_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"yaml_filename": map_file},
                {"frame_id": "map"},
            ],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[configured_params],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[configured_params],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[configured_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[configured_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                        "bt_navigator",
                        "behavior_server",
                    ]
                },
            ],
        ),
    ]

    robot_desc = xacro.process_file(
        os.path.join(
            get_package_share_directory("pmfs_env"),
            "navigation_config",
            "resources",
            "giraff.xacro",
        ),
        mappings={"frame_ns": namespace},
    )
    robot_desc = robot_desc.toprettyxml(indent="  ")

    visualization_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
        ),
    ]

    actions = [PushRosNamespace(namespace)]
    actions.extend(navigation_nodes)
    actions.extend(visualization_nodes)
    return [GroupAction(actions=actions)]


def generate_launch_description():
    my_dir = get_package_share_directory("kdm_vw")

    launch_description = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        DeclareLaunchArgument(
            "nav_params_yaml",
            default_value=os.path.join(my_dir, "navigation_config", "nav2_params.yaml"),
        ),
    ]
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description)
