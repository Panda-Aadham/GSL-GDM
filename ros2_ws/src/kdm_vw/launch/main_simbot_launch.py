import importlib.util
import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def _load_launch_helper(module_name):
    module_path = Path(__file__).with_name(f"{module_name}.py")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_launch_utils = _load_launch_helper("vgr_launch_utils")
_map_utils = _load_launch_helper("vgr_map_utils")

_parse_vgr_simulation_launch = _launch_utils.parse_vgr_simulation_launch
_read_simple_yaml = _launch_utils.read_simple_yaml
_set_launch_configs = _launch_utils.set_launch_configs
_auto_start_position = _map_utils.auto_start_position
_make_navigation_safe_map = _map_utils.make_navigation_safe_map


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="House01"),
        DeclareLaunchArgument("simulation", default_value="1,3-2,4_fast"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_autonomous_exploration", default_value="true"),
        DeclareLaunchArgument("robot_radius", default_value="0.01"),
        DeclareLaunchArgument("close_diagonal_gaps", default_value="False"),
        DeclareLaunchArgument("basic_sim_log_level", default_value="WARN"),
        DeclareLaunchArgument("rviz_log_level", default_value="WARN"),
        DeclareLaunchArgument("player_freq", default_value="1.0"),
        DeclareLaunchArgument("mapper_start_delay", default_value="6.0"),
        DeclareLaunchArgument("explorer_start_delay", default_value="2.0"),
        DeclareLaunchArgument("explorer_waypoint_spacing", default_value="0.8"),
        DeclareLaunchArgument("explorer_clearance", default_value="0.25"),
        DeclareLaunchArgument("explorer_goal_tolerance", default_value="0.30"),
        DeclareLaunchArgument("explorer_goal_timeout", default_value="60.0"),
        DeclareLaunchArgument("explorer_retry_limit", default_value="3"),
        DeclareLaunchArgument("shutdown_on_complete", default_value="true"),
        DeclareLaunchArgument("shutdown_delay", default_value="2.0"),
        DeclareLaunchArgument("kdm_update_period", default_value="1.0"),
        DeclareLaunchArgument("min_observation_spacing", default_value="0.15"),
        DeclareLaunchArgument("min_observation_period", default_value="0.25"),
        DeclareLaunchArgument("observation_check_period", default_value="0.1"),
        DeclareLaunchArgument("max_measurement_age", default_value="2.0"),
    ]


def _truthy(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def _load_simulation_settings(context, pkg_dir, vgr_dir, scenario, simulation):
    local_sim_yaml = Path(pkg_dir) / "scenarios" / scenario / "simulations" / f"{simulation}.yaml"
    if local_sim_yaml.is_file():
        settings = _read_simple_yaml(local_sim_yaml)
    else:
        vgr_launch = Path(vgr_dir) / "scenarios" / scenario / "launch" / simulation / "GADEN_ros2.launch"
        if not vgr_launch.is_file():
            raise RuntimeError(f"Simulation '{simulation}' was not found for scenario '{scenario}'.")
        settings = _parse_vgr_simulation_launch(vgr_launch)

    defaults = {
        "initial_iteration": 280,
        "allow_looping": "true",
        "loop_from_iteration": 280,
        "loop_to_iteration": 288,
    }
    defaults.update(settings)
    settings = defaults
    _set_launch_configs(context, settings)
    return settings


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("kdm_vw")
    vgr_dir = get_package_share_directory("vgr_dataset")
    scenario = LaunchConfiguration("scenario").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    robot_name = LaunchConfiguration("robot_name")

    sim_settings = _load_simulation_settings(context, pkg_dir, vgr_dir, scenario, simulation)
    source_map_file = Path(vgr_dir) / "scenarios" / scenario / "occupancy.yaml"
    runtime_dir = Path(tempfile.gettempdir()) / "kdm_vw" / scenario / simulation
    runtime_dir.mkdir(parents=True, exist_ok=True)

    navigation_map_file = source_map_file
    if _truthy(LaunchConfiguration("close_diagonal_gaps").perform(context)):
        navigation_map_file = runtime_dir / "navigation_occupancy.yaml"
        _make_navigation_safe_map(source_map_file, navigation_map_file, close_diagonal_gaps=True)

    if not all(key in sim_settings for key in ("start_pos_x", "start_pos_y", "start_pos_z")):
        start_x, start_y, start_z = _auto_start_position(
            navigation_map_file,
            sim_settings["source_x"],
            sim_settings["source_y"],
            min_clearance=max(float(LaunchConfiguration("robot_radius").perform(context)) * 2.0, 0.2),
        )
        sim_settings["start_pos_x"] = start_x
        sim_settings["start_pos_y"] = start_y
        sim_settings["start_pos_z"] = start_z
        _set_launch_configs(context, {"start_pos_x": start_x, "start_pos_y": start_y, "start_pos_z": start_z})

    world_file = runtime_dir / f"basic_sim_{scenario}.yaml"
    world_file.write_text(
        "\n".join([
            f'map: "{navigation_map_file}"',
            "robots:",
            '  - name: "PioneerP3DX"',
            f"    radius: {LaunchConfiguration('robot_radius').perform(context)}",
            f"    position: [{sim_settings['start_pos_x']}, {sim_settings['start_pos_y']}, {sim_settings['start_pos_z']}]",
            "    angle: 0.0",
            "    publishOdom: true",
            "    publishMapToOdomTF: true",
            "    sensors:",
            '      - type: "laser"',
            '        name: "laser_scan"',
            "        minAngleRad: -0.7",
            "        maxAngleRad: 3.8",
            "        angleResolutionRad: 0.1",
            "        minDistance: 0.1",
            "        maxDistance: 4.0",
            "",
        ]),
        encoding="utf-8",
    )

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", "gaden_player_launch.py")),
        launch_arguments={
            "use_rviz": "False",
            "scenario": scenario,
            "simulation": simulation,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, "navigation_config", "nav2_launch.py")),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
        launch_arguments={
            "scenario": scenario,
            "namespace": robot_name,
            "map_file": str(navigation_map_file),
            "robot_radius": LaunchConfiguration("robot_radius"),
        }.items(),
    )

    basic_sim = Node(
        package="basic_sim",
        executable="basic_sim",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("basic_sim_log_level")],
        parameters=[
            {"deltaTime": 0.1},
            {"speed": 5.0},
            {"worldFile": str(world_file)},
        ],
    )

    delayed_nodes = TimerAction(
        period=float(LaunchConfiguration("mapper_start_delay").perform(context)),
        actions=[
            GroupAction(actions=[
                PushRosNamespace(robot_name),
                Node(
                    package="simulated_anemometer",
                    executable="simulated_anemometer",
                    name="Anemometer",
                    output="screen",
                    parameters=[
                        {"sensor_frame": "PioneerP3DX_anemometer_frame"},
                        {"fixed_frame": "map"},
                        {"noise_std": 0.3},
                        {"use_map_ref_system": True},
                    ],
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="anemometer_tf_pub",
                    arguments=[
                        "--x", "0", "--y", "0", "--z", "0.5",
                        "--qx", "1.0", "--qy", "0.0", "--qz", "0.0", "--qw", "0.0",
                        "--frame-id", "PioneerP3DX_base_link",
                        "--child-frame-id", "PioneerP3DX_anemometer_frame",
                    ],
                    output="screen",
                ),
                Node(
                    package="simulated_gas_sensor",
                    executable="simulated_gas_sensor",
                    name="PID",
                    output="screen",
                    parameters=[
                        {"sensor_model": 30},
                        {"sensor_frame": "PioneerP3DX_pid_frame"},
                        {"fixed_frame": "map"},
                        {"noise_std": 20.1},
                    ],
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="pid_tf_pub",
                    arguments=[
                        "--x", "0", "--y", "0", "--z", "0.5",
                        "--qx", "1.0", "--qy", "0.0", "--qz", "0.0", "--qw", "0.0",
                        "--frame-id", "PioneerP3DX_base_link",
                        "--child-frame-id", "PioneerP3DX_pid_frame",
                    ],
                    output="screen",
                ),
                Node(
                    package="kdm_vw",
                    executable="kdm_vw_mapper",
                    name="KDM_VW",
                    output="screen",
                    parameters=[
                        {"scenario": scenario},
                        {"simulation": simulation},
                        {"occupancy_yaml": str(navigation_map_file)},
                        {"update_period": float(LaunchConfiguration("kdm_update_period").perform(context))},
                        {"min_observation_spacing": float(LaunchConfiguration("min_observation_spacing").perform(context))},
                        {"min_observation_period": float(LaunchConfiguration("min_observation_period").perform(context))},
                        {"observation_check_period": float(LaunchConfiguration("observation_check_period").perform(context))},
                        {"max_measurement_age": float(LaunchConfiguration("max_measurement_age").perform(context))},
                        {"output_dir": "results/KDM_VW"},
                    ],
                ),
            ]),
            GroupAction(
                condition=IfCondition(LaunchConfiguration("use_autonomous_exploration")),
                actions=[
                    PushRosNamespace(robot_name),
                    Node(
                        package="kdm_vw",
                        executable="coverage_explorer",
                        name="Explorer",
                        output="screen",
                        on_exit=[
                            LogInfo(msg="[VGR KDM_VW] Coverage explorer exited. Shutting down launch."),
                            Shutdown(reason="Coverage explorer completed"),
                        ] if _truthy(LaunchConfiguration("shutdown_on_complete").perform(context)) else [],
                        parameters=[
                            {"occupancy_yaml": str(navigation_map_file)},
                            {"start_delay": float(LaunchConfiguration("explorer_start_delay").perform(context))},
                            {"waypoint_spacing": float(LaunchConfiguration("explorer_waypoint_spacing").perform(context))},
                            {"clearance": float(LaunchConfiguration("explorer_clearance").perform(context))},
                            {"goal_tolerance": float(LaunchConfiguration("explorer_goal_tolerance").perform(context))},
                            {"goal_timeout": float(LaunchConfiguration("explorer_goal_timeout").perform(context))},
                            {"retry_limit": int(LaunchConfiguration("explorer_retry_limit").perform(context))},
                            {"shutdown_on_complete": _truthy(LaunchConfiguration("shutdown_on_complete").perform(context))},
                            {"shutdown_delay": float(LaunchConfiguration("shutdown_delay").perform(context))},
                        ],
                    ),
                ],
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_dir, "launch", "explorer.rviz"),
            "--ros-args",
            "--log-level",
            LaunchConfiguration("rviz_log_level"),
        ],
    )

    actions = [
        LogInfo(msg=[f"[VGR KDM_VW] Scenario={scenario} Simulation={simulation}"]),
        LogInfo(msg=[f"[VGR KDM_VW] Navigation map: {navigation_map_file}"]),
        LogInfo(msg=[f"[VGR KDM_VW] Runtime dir: {runtime_dir}"]),
        gaden_player,
        nav2,
        basic_sim,
        delayed_nodes,
    ]
    if _truthy(LaunchConfiguration("use_rviz").perform(context)):
        actions.append(LogInfo(msg=[f"[VGR KDM_VW] Launching RViz config: {os.path.join(pkg_dir, 'launch', 'explorer.rviz')}"]))
        actions.append(rviz)
    else:
        actions.append(LogInfo(msg="[VGR KDM_VW] RViz disabled by use_rviz:=false"))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory("kdm_vw")
    actions = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration("pkg_dir", pkg_dir),
        SetLaunchConfiguration("robot_name", "PioneerP3DX"),
        SetLaunchConfiguration("nav_params_yaml", PathJoinSubstitution([pkg_dir, "navigation_config", "nav2_params.yaml"])),
    ]
    actions.extend(launch_arguments())
    actions.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(actions)
