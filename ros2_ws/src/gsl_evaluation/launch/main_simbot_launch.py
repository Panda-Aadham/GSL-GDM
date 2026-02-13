import importlib.util
import os
import tempfile
from pathlib import Path

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
from launch.frontend.parse_substitution import parse_substitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def _load_launch_helper(module_name):
    module_path = Path(__file__).with_name(f"{module_name}.py")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_gsl_method_config = _load_launch_helper("gsl_method_config")
_launch_utils = _load_launch_helper("vgr_launch_utils")
_map_utils = _load_launch_helper("vgr_map_utils")

NON_SEMANTIC_METHODS = _gsl_method_config.NON_SEMANTIC_METHODS
SEMANTIC_METHODS = _gsl_method_config.SEMANTIC_METHODS
method_defaults = _gsl_method_config.method_defaults

_parse_vgr_simulation_launch = _launch_utils.parse_vgr_simulation_launch
_read_simple_yaml = _launch_utils.read_simple_yaml
_set_launch_configs = _launch_utils.set_launch_configs
_next_csv_run_index = _launch_utils.next_csv_run_index

_auto_start_position = _map_utils.auto_start_position
_make_navigation_safe_map = _map_utils.make_navigation_safe_map
_suggest_dynamic_convergence_threshold = _map_utils.suggest_dynamic_convergence_threshold


def _truthy(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def _auto_bool(value, default):
    value = str(value).strip()
    if value.lower() in ("", "auto"):
        return bool(default)
    return _truthy(value)


def _auto_float(value, default):
    value = str(value).strip()
    if value.lower() in ("", "auto"):
        return float(default)
    return float(value)


def loud_logs(title, details=None, condition=None):
    actions = [
        LogInfo(condition=condition, msg=""),
        LogInfo(condition=condition, msg="================================================================"),
        LogInfo(condition=condition, msg=["[VGR GSL] ", title]),
    ]
    if details:
        for detail in details:
            actions.append(LogInfo(condition=condition, msg=["  ", *detail]))
    actions.append(LogInfo(condition=condition, msg="================================================================"))
    return actions


def _load_simulation_settings(context, pkg_dir, vgr_dir, scenario, simulation):
    local_sim_yaml = Path(pkg_dir) / "scenarios" / scenario / "simulations" / f"{simulation}.yaml"
    if local_sim_yaml.is_file():
        settings = _read_simple_yaml(local_sim_yaml)
    else:
        vgr_launch = Path(vgr_dir) / "scenarios" / scenario / "launch" / simulation / "GADEN_ros2.launch"
        if not vgr_launch.is_file():
            available = sorted(path.name for path in (Path(vgr_dir) / "scenarios" / scenario / "gas_simulations").glob("*") if path.is_dir())
            raise RuntimeError(
                f"Simulation '{simulation}' was not found for {scenario}. "
                f"Available simulations: {', '.join(available)}"
            )
        settings = _parse_vgr_simulation_launch(vgr_launch)

    required = ("source_x", "source_y", "source_z", "gas_type")
    missing = [name for name in required if name not in settings]
    if missing:
        raise RuntimeError(f"Simulation settings for {scenario}/{simulation} are missing: {', '.join(missing)}")

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


def _make_rviz_config_compatible(source_config, target_config):
    text = source_config.read_text(encoding="utf-8")
    # The original PMFS RViz configs reference an optional nav_assistant_tools
    # plugin that is not present in a standard Nav2 RViz install.
    text = text.replace("nav_assistant_tools/SetNavGoal", "rviz_default_plugins/SetGoal")
    target_config.write_text(text, encoding="utf-8")
    return target_config


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="House01"),
        DeclareLaunchArgument("simulation", default_value="1,3-2,4_fast"),
        DeclareLaunchArgument("method", default_value="PMFS"),
        DeclareLaunchArgument("use_infotaxis", default_value="auto"),
        DeclareLaunchArgument("stop_and_measure_time", default_value="auto"),
        DeclareLaunchArgument("th_gas_present", default_value="auto"),
        DeclareLaunchArgument("th_wind_present", default_value="auto"),
        DeclareLaunchArgument("max_wait_for_gas_time", default_value="auto"),
        DeclareLaunchArgument("global_exploration_on_gas_timeout", default_value="auto"),
        DeclareLaunchArgument("grgsl_global_move_fallback", default_value="auto"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("use_hit_rviz", default_value="True"),
        DeclareLaunchArgument("use_source_rviz", default_value="True"),
        DeclareLaunchArgument("gsl_scale", default_value="1"),
        DeclareLaunchArgument("gsl_map_resolution", default_value="0.16"),
        DeclareLaunchArgument("convergence_thr", default_value=""),
        DeclareLaunchArgument("robot_radius", default_value="0.01"),
        DeclareLaunchArgument("close_diagonal_gaps", default_value="False"),
        DeclareLaunchArgument("basic_sim_log_level", default_value="WARN"),
        DeclareLaunchArgument("rviz_log_level", default_value="WARN"),
        DeclareLaunchArgument("gsl_start_delay", default_value="6.0"),
        DeclareLaunchArgument("gsl_call_delay", default_value="12.0"),
        DeclareLaunchArgument("dynamic_threshold_target_error", default_value="1.0"),
        DeclareLaunchArgument("dynamic_threshold_alpha", default_value="0.03"),
        DeclareLaunchArgument("dynamic_threshold_max_error", default_value="1.5"),
        DeclareLaunchArgument("variance_log_interval", default_value="5.0"),
        DeclareLaunchArgument("run_index", default_value="auto"),
    ]


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name")
    pkg_dir = get_package_share_directory("vgr_pmfs_house01_env")
    wrapper_launch_dir = Path(pkg_dir) / "launch"
    vgr_dir = get_package_share_directory("vgr_dataset")
    scenario = LaunchConfiguration("scenario").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    method = LaunchConfiguration("method").perform(context)
    source_map_file = Path(vgr_dir) / "scenarios" / scenario / "occupancy.yaml"
    if not source_map_file.is_file():
        raise RuntimeError(f"Scenario '{scenario}' was not found in the VGR dataset.")

    sim_settings = _load_simulation_settings(context, pkg_dir, vgr_dir, scenario, simulation)
    gsl_scale = int(LaunchConfiguration("gsl_scale").perform(context))
    gsl_map_resolution_value = LaunchConfiguration("gsl_map_resolution").perform(context).strip()
    gsl_map_resolution = float(gsl_map_resolution_value) if gsl_map_resolution_value else None
    convergence_thr = LaunchConfiguration("convergence_thr").perform(context).strip()
    robot_radius = LaunchConfiguration("robot_radius").perform(context)
    close_diagonal_gaps = _truthy(LaunchConfiguration("close_diagonal_gaps").perform(context))
    gsl_start_delay = float(LaunchConfiguration("gsl_start_delay").perform(context))
    gsl_call_delay = float(LaunchConfiguration("gsl_call_delay").perform(context))
    dynamic_threshold_target_error = float(LaunchConfiguration("dynamic_threshold_target_error").perform(context))
    dynamic_threshold_alpha = float(LaunchConfiguration("dynamic_threshold_alpha").perform(context))
    dynamic_threshold_max_error = float(LaunchConfiguration("dynamic_threshold_max_error").perform(context))
    variance_log_interval = float(LaunchConfiguration("variance_log_interval").perform(context))
    run_index_value = LaunchConfiguration("run_index").perform(context).strip()
    if method in SEMANTIC_METHODS:
        raise RuntimeError(
            f"Method '{method}' requires the semantics stack and extra ontology/detection inputs. "
            "The VGR wrapper currently supports non-semantic methods only."
        )
    if method not in NON_SEMANTIC_METHODS:
        raise RuntimeError(
            f"Unsupported method '{method}'. Supported methods for this wrapper are: "
            + ", ".join(sorted(NON_SEMANTIC_METHODS))
        )
    if gsl_scale < 1:
        raise RuntimeError(
            f"gsl_scale must be a positive integer. Got gsl_scale={gsl_scale}."
        )
    if gsl_map_resolution is not None and gsl_map_resolution <= 0:
        raise RuntimeError(
            f"gsl_map_resolution must be positive when provided. Got {gsl_map_resolution}."
        )
    if gsl_start_delay < 0 or gsl_call_delay < 0:
        raise RuntimeError(
            f"gsl_start_delay and gsl_call_delay must be non-negative. Got {gsl_start_delay} and {gsl_call_delay}."
        )
    if dynamic_threshold_target_error <= 0 or dynamic_threshold_alpha < 0 or dynamic_threshold_max_error <= 0:
        raise RuntimeError(
            "dynamic_threshold_target_error and dynamic_threshold_max_error must be positive, "
            "and dynamic_threshold_alpha must be non-negative."
        )
    if dynamic_threshold_target_error > dynamic_threshold_max_error:
        raise RuntimeError(
            "dynamic_threshold_target_error must be less than or equal to dynamic_threshold_max_error."
        )
    if variance_log_interval < 0:
        raise RuntimeError(
            f"variance_log_interval must be zero or positive. Got {variance_log_interval}."
        )
    if run_index_value.lower() not in ("", "auto") and int(run_index_value) < 1:
        raise RuntimeError(f"run_index must be a positive integer or 'auto'. Got {run_index_value}.")

    defaults = method_defaults(method)
    use_infotaxis = _auto_bool(
        LaunchConfiguration("use_infotaxis").perform(context),
        defaults["default_use_infotaxis"],
    )
    stop_and_measure_time = _auto_float(
        LaunchConfiguration("stop_and_measure_time").perform(context),
        defaults["stop_and_measure_time"],
    )
    th_gas_present = _auto_float(
        LaunchConfiguration("th_gas_present").perform(context),
        defaults["th_gas_present"],
    )
    th_wind_present = _auto_float(
        LaunchConfiguration("th_wind_present").perform(context),
        defaults["th_wind_present"],
    )
    max_wait_for_gas_time = _auto_float(
        LaunchConfiguration("max_wait_for_gas_time").perform(context),
        defaults["max_wait_for_gas_time"],
    )
    global_exploration_on_gas_timeout = _auto_bool(
        LaunchConfiguration("global_exploration_on_gas_timeout").perform(context),
        defaults["global_exploration_on_gas_timeout"],
    )
    grgsl_global_move_fallback = _auto_bool(
        LaunchConfiguration("grgsl_global_move_fallback").perform(context),
        defaults["grgsl_global_move_fallback"],
    )
    convergence_thr_display = convergence_thr if convergence_thr else "algorithm default"
    results_dir = Path("results") / method
    results_dir.mkdir(parents=True, exist_ok=True)
    results_file = results_dir / f"{scenario}_{simulation}.csv"
    variance_log_file = results_dir / f"{scenario}_{simulation}_variance.csv"
    run_index = (
        max(
            _next_csv_run_index(results_file),
            _next_csv_run_index(variance_log_file),
        )
        if run_index_value.lower() in ("", "auto")
        else int(run_index_value)
    )

    runtime_dir = Path(tempfile.gettempdir()) / "vgr_pmfs_house01_env" / scenario / simulation
    runtime_dir.mkdir(parents=True, exist_ok=True)
    rviz_hit_config = _make_rviz_config_compatible(
        wrapper_launch_dir / "hit.rviz",
        runtime_dir / "hit_compatible.rviz",
    )
    rviz_source_config = _make_rviz_config_compatible(
        wrapper_launch_dir / "source.rviz",
        runtime_dir / "source_compatible.rviz",
    )
    navigation_map_file = runtime_dir / "navigation_occupancy.yaml"
    source_map_metadata = _read_simple_yaml(source_map_file)
    source_map_resolution = float(source_map_metadata.get("resolution", "0.1"))
    map_stats = {
        "blocked_diagonal_cells": 0,
        "source_resolution": source_map_resolution,
        "output_resolution": source_map_resolution,
        "source_size": "original",
        "output_size": "original",
    }
    if close_diagonal_gaps or gsl_map_resolution is not None:
        map_stats = _make_navigation_safe_map(
            source_map_file,
            navigation_map_file,
            target_resolution=gsl_map_resolution,
            close_diagonal_gaps=close_diagonal_gaps,
        )
    else:
        navigation_map_file = source_map_file
    effective_gsl_cell_size = map_stats["output_resolution"] * gsl_scale
    dynamic_threshold_stats = _suggest_dynamic_convergence_threshold(
        navigation_map_file,
        target_error=dynamic_threshold_target_error,
        alpha=dynamic_threshold_alpha,
        max_error=dynamic_threshold_max_error,
    )

    if not all(key in sim_settings for key in ("start_pos_x", "start_pos_y", "start_pos_z")):
        auto_start_clearance = max(float(robot_radius) * 2.0, map_stats["output_resolution"] * 2.0)
        start_x, start_y, start_z = _auto_start_position(
            navigation_map_file,
            sim_settings["source_x"],
            sim_settings["source_y"],
            min_clearance=auto_start_clearance,
        )
        sim_settings["start_pos_x"] = start_x
        sim_settings["start_pos_y"] = start_y
        sim_settings["start_pos_z"] = start_z
        _set_launch_configs(
            context,
            {
                "start_pos_x": start_x,
                "start_pos_y": start_y,
                "start_pos_z": start_z,
            },
        )
    else:
        auto_start_clearance = 0.0

    start_pos_x = sim_settings["start_pos_x"]
    start_pos_y = sim_settings["start_pos_y"]
    start_pos_z = sim_settings["start_pos_z"]

    world_file = runtime_dir / f"basic_sim_{scenario}.yaml"
    world_file.write_text(
        "\n".join([
            f'map: "{navigation_map_file}"',
            "robots:",
            '  - name: "PioneerP3DX"',
            f"    radius: {robot_radius}",
            f"    position: [{start_pos_x}, {start_pos_y}, {start_pos_z}]",
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

    hit_rviz_condition = IfCondition(LaunchConfiguration("use_hit_rviz"))
    source_rviz_condition = IfCondition(LaunchConfiguration("use_source_rviz"))

    gsl_call = [
        GroupAction(actions=[
            PushRosNamespace(robot_name),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_call",
                name="gsl_call",
                output="screen",
                parameters=[
                    {"method": parse_substitution("$(var method)")},
                ],
            ),
        ])
    ]

    gsl_parameters = [
        {"use_sim_time": False},
        {"maxSearchTime": 300.0},
        {"robot_location_topic": "ground_truth"},
        {"stop_and_measure_time": stop_and_measure_time},
        {"th_gas_present": th_gas_present},
        {"th_wind_present": th_wind_present},
        {"maxWaitForGasTime": max_wait_for_gas_time},
        {"global_exploration_on_gas_timeout": global_exploration_on_gas_timeout},
        {"grgsl_global_move_fallback": grgsl_global_move_fallback},
        {"ground_truth_x": parse_substitution("$(var source_x)")},
        {"ground_truth_y": parse_substitution("$(var source_y)")},
        {"resultsFile": parse_substitution("results/$(var method)/$(var scenario)_$(var simulation).csv")},
        {"varianceLogFile": str(variance_log_file)},
        {"varianceLogInterval": variance_log_interval},
        {"varianceLogRunIndex": run_index},
        {"scale": parse_substitution("$(var gsl_scale)")},
        {"markers_height": defaults["markers_height"]},
        {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
        {"openMoveSetExpasion": defaults["openMoveSetExpasion"]},
        {"explorationProbability": defaults["explorationProbability"]},
        {"useDiffusionTerm": defaults["useDiffusionTerm"]},
        {"stdevHit": defaults["stdevHit"]},
        {"stdevMiss": defaults["stdevMiss"]},
        {"infoTaxis": use_infotaxis},
        {"allowMovementRepetition": use_infotaxis if method == "PMFS" else True},
        {"headless": defaults["headless"]},
        {"distanceWeight": defaults["distanceWeight"]},
        {"maxUpdatesPerStop": defaults["maxUpdatesPerStop"]},
        {"kernelSigma": defaults["kernelSigma"]},
        {"kernelStretchConstant": defaults["kernelStretchConstant"]},
        {"hitPriorProbability": defaults["hitPriorProbability"]},
        {"confidenceSigmaSpatial": defaults["confidenceSigmaSpatial"]},
        {"confidenceMeasurementWeight": defaults["confidenceMeasurementWeight"]},
        {"initialExplorationMoves": defaults["initialExplorationMoves"]},
        {"useWindGroundTruth": defaults["useWindGroundTruth"]},
        {"stepsSourceUpdate": defaults["stepsSourceUpdate"]},
        {"maxRegionSize": defaults["maxRegionSize"]},
        {"sourceDiscriminationPower": parse_substitution("$(var sourceDiscriminationPower)")},
        {"refineFraction": defaults["refineFraction"]},
        {"deltaTime": parse_substitution("$(var filamentDeltaTime)")},
        {"noiseSTDev": parse_substitution("$(var filament_movement_stdev)")},
        {"iterationsToRecord": parse_substitution("$(var iterationsToRecord)")},
        {"minWarmupIterations": parse_substitution("$(var minWarmupIterations)")},
        {"maxWarmupIterations": parse_substitution("$(var maxWarmupIterations)")},
        {"blurSigmaX": defaults["blurSigmaX"]},
        {"blurSigmaY": defaults["blurSigmaY"]},
        {"step": defaults["step"]},
        {"initial_step": defaults["initial_step"]},
        {"step_increment": defaults["step_increment"]},
        {"Kmu": defaults["Kmu"]},
        {"Kp": defaults["Kp"]},
        {"intervalLength": defaults["intervalLength"]},
        {"initSpiralStep": defaults["initSpiralStep"]},
        {"spiralStep_increment": defaults["spiralStep_increment"]},
        {"numberOfParticles": defaults["numberOfParticles"]},
        {"maxEstimations": defaults["maxEstimations"]},
        {"numberOfWindObs": defaults["numberOfWindObs"]},
        {"convergenceThr": defaults["convergenceThr"]},
        {"deltaT": defaults["deltaT"]},
        {"mu": defaults["mu"]},
        {"Sp": defaults["Sp"]},
        {"Rconv": defaults["Rconv"]},
    ]
    if convergence_thr:
        gsl_parameters.append({"convergence_thr": float(convergence_thr)})

    gsl_node = [
        GroupAction(actions=[
            PushRosNamespace(robot_name),
            Node(
                package="gsl_server",
                executable="gsl_actionserver_node",
                name="GSL",
                output="screen",
                parameters=gsl_parameters,
                on_exit=Shutdown(),
            ),
        ])
    ]

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

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "gaden_player_launch.py")
        ),
        launch_arguments={
            "use_rviz": "False",
            "scenario": LaunchConfiguration("scenario").perform(context),
            "simulation": LaunchConfiguration("simulation").perform(context),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "navigation_config", "nav2_launch.py")
        ),
        launch_arguments={
            "scenario": LaunchConfiguration("scenario"),
            "namespace": robot_name,
            "map_file": str(navigation_map_file),
            "robot_radius": LaunchConfiguration("robot_radius"),
        }.items(),
    )

    anemometer = [
        GroupAction(actions=[
            PushRosNamespace(robot_name),
            Node(
                package="simulated_anemometer",
                executable="simulated_anemometer",
                name="Anemometer",
                output="screen",
                parameters=[
                    {"sensor_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 0.3},
                    {"use_map_ref_system": False},
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="anemometer_tf_pub",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0.5",
                    "--qx", "1.0",
                    "--qy", "0.0",
                    "--qz", "0.0",
                    "--qw", "0.0",
                    "--frame-id", parse_substitution("$(var robot_name)_base_link"),
                    "--child-frame-id", parse_substitution("$(var robot_name)_anemometer_frame"),
                ],
                output="screen",
            ),
        ])
    ]

    pid = [
        GroupAction(actions=[
            PushRosNamespace(robot_name),
            Node(
                package="simulated_gas_sensor",
                executable="simulated_gas_sensor",
                name="PID",
                output="screen",
                parameters=[
                    {"sensor_model": 30},
                    {"sensor_frame": parse_substitution("$(var robot_name)_pid_frame")},
                    {"fixed_frame": "map"},
                    {"noise_std": 20.1},
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="pid_tf_pub",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0.5",
                    "--qx", "1.0",
                    "--qy", "0.0",
                    "--qz", "0.0",
                    "--qw", "0.0",
                    "--frame-id", parse_substitution("$(var robot_name)_base_link"),
                    "--child-frame-id", parse_substitution("$(var robot_name)_pid_frame"),
                ],
                output="screen",
            ),
        ])
    ]

    rviz_hit_log = LogInfo(
        condition=hit_rviz_condition,
        msg=["Launching RViz config: ", str(rviz_hit_config)],
    )

    rviz_hit = Node(
        condition=hit_rviz_condition,
        package="rviz2",
        executable="rviz2",
        name="rviz_hit",
        output="screen",
        arguments=[
            "-d",
            str(rviz_hit_config),
            "--ros-args",
            "--log-level",
            LaunchConfiguration("rviz_log_level"),
        ],
    )

    rviz_source_log = LogInfo(
        condition=source_rviz_condition,
        msg=["Launching RViz config: ", str(rviz_source_config)],
    )

    rviz_source = Node(
        condition=source_rviz_condition,
        package="rviz2",
        executable="rviz2",
        name="rviz_source",
        output="screen",
        arguments=[
            "-d",
            str(rviz_source_config),
            "--ros-args",
            "--log-level",
            LaunchConfiguration("rviz_log_level"),
        ],
    )

    delayed_sensor_and_server_nodes = TimerAction(
        period=gsl_start_delay,
        actions=[
            *loud_logs(
                "STARTING SENSORS AND GSL SERVER",
                details=[
                    ["robot namespace:", robot_name],
                    ["gas topic:", "/", robot_name, "/PID/Sensor_reading"],
                    ["wind topic:", "/", robot_name, "/Anemometer/WindSensor_reading"],
                    ["localization topic:", "/", robot_name, "/ground_truth"],
                    ["navigation map:", str(navigation_map_file)],
                    ["robot_radius:", LaunchConfiguration("robot_radius")],
                ],
            ),
            *anemometer,
            *pid,
            *gsl_node,
        ],
    )

    delayed_gsl_call = TimerAction(
        period=gsl_call_delay,
        actions=[
            *loud_logs(
                "SENDING GOAL TO GSL ACTION SERVER",
                details=[
                    ["method:", LaunchConfiguration("method")],
                    ["scenario:", LaunchConfiguration("scenario")],
                    ["simulation:", LaunchConfiguration("simulation")],
                    ["gsl_scale:", LaunchConfiguration("gsl_scale")],
                    ["gsl_map_resolution:", f"{map_stats['output_resolution']:g}"],
                    ["effective GSL cell:", f"{effective_gsl_cell_size:g} m"],
                    ["convergence_thr:", convergence_thr_display],
                    ["use_infotaxis:", str(use_infotaxis)],
                    ["th_gas_present:", f"{th_gas_present:g}"],
                    ["max_wait_for_gas_time:", f"{max_wait_for_gas_time:g}"],
                    ["global exploration on gas timeout:", str(global_exploration_on_gas_timeout)],
                    ["grgsl global move fallback:", str(grgsl_global_move_fallback)],
                    ["suggested dynamic convergence_thr:", f"{dynamic_threshold_stats['suggested_threshold']:.3f}"],
                ],
            ),
            *gsl_call,
        ],
    )

    actions = []
    actions.extend(loud_logs(
        "PREPARING VGR GSL WRAPPER",
        details=[
            ["scenario:", LaunchConfiguration("scenario")],
            ["simulation:", LaunchConfiguration("simulation")],
            ["method:", LaunchConfiguration("method")],
            ["source:", sim_settings["source_x"], ", ", sim_settings["source_y"], ", ", sim_settings["source_z"]],
            ["start:", sim_settings["start_pos_x"], ", ", sim_settings["start_pos_y"], ", ", sim_settings["start_pos_z"]],
            ["auto start clearance:", f"{auto_start_clearance:g} m" if auto_start_clearance else "manual/local"],
            ["robot:", robot_name],
            ["runtime dir:", str(runtime_dir)],
            ["results file:", str(results_file)],
            ["variance log file:", str(variance_log_file)],
            ["variance log interval:", f"{variance_log_interval:g} s"],
            ["run index:", str(run_index)],
            ["world file:", str(world_file)],
            ["navigation map:", str(navigation_map_file)],
            ["hit rviz config:", str(rviz_hit_config)],
            ["source rviz config:", str(rviz_source_config)],
            ["map resolution:", f"{map_stats['source_resolution']:g}", " -> ", f"{map_stats['output_resolution']:g}"],
            ["map size:", map_stats["source_size"], " -> ", map_stats["output_size"]],
            ["effective GSL cell:", f"{effective_gsl_cell_size:g} m"],
            ["diagonal cells blocked:", str(map_stats["blocked_diagonal_cells"])],
            ["use_infotaxis:", str(use_infotaxis)],
            ["stop_and_measure_time:", f"{stop_and_measure_time:g}"],
            ["th_gas_present:", f"{th_gas_present:g}"],
            ["th_wind_present:", f"{th_wind_present:g}"],
            ["max_wait_for_gas_time:", f"{max_wait_for_gas_time:g}"],
            ["global exploration on gas timeout:", str(global_exploration_on_gas_timeout)],
            ["grgsl global move fallback:", str(grgsl_global_move_fallback)],
        ],
    ))
    actions.extend(loud_logs(
        "DYNAMIC GSL CONVERGENCE THRESHOLD SUGGESTION",
        details=[
            ["note:", "advisory only; pass convergence_thr:=VALUE to use it"],
            ["active convergence_thr:", convergence_thr_display],
            ["suggested convergence_thr:", f"{dynamic_threshold_stats['suggested_threshold']:.3f}"],
            ["suggested uncertainty radius:", f"{dynamic_threshold_stats['suggested_uncertainty_radius']:.3f} m"],
            ["formula:", "min(max(target_error^2, alpha * uniform_map_variance), max_error^2)"],
            ["target_error:", f"{dynamic_threshold_stats['target_error']:.3f} m"],
            ["alpha:", f"{dynamic_threshold_stats['alpha']:.3f}"],
            ["max_error:", f"{dynamic_threshold_stats['max_error']:.3f} m"],
            ["uniform map variance:", f"{dynamic_threshold_stats['uniform_variance']:.3f} m^2"],
            ["uniform map std:", f"{dynamic_threshold_stats['uniform_std']:.3f} m"],
            ["map contribution:", f"{dynamic_threshold_stats['map_threshold']:.3f}"],
            ["free cells:", str(dynamic_threshold_stats["free_cells"])],
            ["free area:", f"{dynamic_threshold_stats['free_area']:.3f} m^2"],
            ["example CLI:", "convergence_thr:=", f"{dynamic_threshold_stats['suggested_threshold']:.3f}"],
        ],
    ))
    actions.extend(loud_logs(
        "LAUNCHING CORE STACK",
        details=[
            ["gaden player:", "enabled"],
            ["nav2:", "enabled"],
            ["basic_sim:", "enabled"],
            ["supported methods:", ", ".join(sorted(NON_SEMANTIC_METHODS))],
            ["gsl_scale:", LaunchConfiguration("gsl_scale")],
            ["gsl_map_resolution:", f"{map_stats['output_resolution']:g}"],
            ["effective GSL cell:", f"{effective_gsl_cell_size:g} m"],
            ["convergence_thr:", convergence_thr_display],
            ["suggested convergence_thr:", f"{dynamic_threshold_stats['suggested_threshold']:.3f}"],
            ["use_infotaxis:", str(use_infotaxis)],
            ["stop_and_measure_time:", f"{stop_and_measure_time:g}"],
            ["th_gas_present:", f"{th_gas_present:g}"],
            ["th_wind_present:", f"{th_wind_present:g}"],
            ["max_wait_for_gas_time:", f"{max_wait_for_gas_time:g}"],
            ["global exploration on gas timeout:", str(global_exploration_on_gas_timeout)],
            ["grgsl global move fallback:", str(grgsl_global_move_fallback)],
            ["variance log file:", str(variance_log_file)],
            ["variance log interval:", f"{variance_log_interval:g} s"],
            ["run index:", str(run_index)],
            ["robot_radius:", LaunchConfiguration("robot_radius")],
            ["close_diagonal_gaps:", LaunchConfiguration("close_diagonal_gaps")],
            ["basic_sim_log_level:", LaunchConfiguration("basic_sim_log_level")],
            ["rviz_log_level:", LaunchConfiguration("rviz_log_level")],
            ["gsl_start_delay:", LaunchConfiguration("gsl_start_delay")],
            ["gsl_call_delay:", LaunchConfiguration("gsl_call_delay")],
            ["use_rviz:", LaunchConfiguration("use_rviz")],
            ["rviz hit:", LaunchConfiguration("use_hit_rviz")],
            ["rviz source:", LaunchConfiguration("use_source_rviz")],
        ],
    ))
    actions.append(gaden_player)
    actions.append(nav2)
    actions.append(basic_sim)
    actions.append(delayed_sensor_and_server_nodes)
    actions.append(delayed_gsl_call)
    if LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1", "yes") and defaults["use_gsl_rviz"]:
        actions.extend(loud_logs(
            "OPENING HIT RVIZ WINDOW",
            details=[
                ["config:", str(wrapper_launch_dir / "hit.rviz")],
                ["runtime config:", str(rviz_hit_config)],
            ],
            condition=hit_rviz_condition,
        ))
        actions.append(rviz_hit_log)
        actions.append(rviz_hit)
        actions.extend(loud_logs(
            "OPENING SOURCE RVIZ WINDOW",
            details=[
                ["config:", str(wrapper_launch_dir / "source.rviz")],
                ["runtime config:", str(rviz_source_config)],
            ],
            condition=source_rviz_condition,
        ))
        actions.append(rviz_source_log)
        actions.append(rviz_source)
    elif LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1", "yes"):
        actions.extend(loud_logs(
            "METHOD DOES NOT USE HIT/SOURCE RVIZ",
            details=[
                ["method:", method],
                ["reason:", "hit/source RViz is enabled for supported non-semantic GSL methods only"],
            ],
        ))
    else:
        actions.extend(loud_logs(
            "RVIZ DISABLED",
            details=[
                ["reason:", "use_rviz launch argument is false"],
            ],
        ))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory("vgr_pmfs_house01_env")

    launch_description = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(name="pkg_dir", value=[pkg_dir]),
        SetLaunchConfiguration(
            name="nav_params_yaml",
            value=[PathJoinSubstitution([pkg_dir, "navigation_config", "nav2_params.yaml"])],
        ),
        SetLaunchConfiguration(name="robot_name", value="PioneerP3DX"),
        SetLaunchConfiguration(name="player_freq", value="0.1"),
        SetLaunchConfiguration(name="filament_movement_stdev", value="0.5"),
        SetLaunchConfiguration(name="sourceDiscriminationPower", value="0.3"),
        SetLaunchConfiguration(name="iterationsToRecord", value="200"),
        SetLaunchConfiguration(name="minWarmupIterations", value="0"),
        SetLaunchConfiguration(name="maxWarmupIterations", value="500"),
        SetLaunchConfiguration(name="filamentDeltaTime", value="0.1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description)
