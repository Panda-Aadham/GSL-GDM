import os
import math
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution

import sys
sys.path.append(get_package_share_directory("gaden_common"))
from gaden_internal_py.utils import read_sim_yaml  # NOQA # type: ignore


NON_SEMANTIC_METHODS = {
    "PMFS",
    "GrGSL",
    "ParticleFilter",
    "Spiral",
    "SurgeCast",
    "SurgeSpiral",
}

SEMANTIC_METHODS = {
    "SemanticPMFS",
    "SemanticGrGSL",
}

PMFS_STYLE_METHODS = {
    "PMFS",
}


def _truthy(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def loud_logs(title, details=None, condition=None):
    actions = [
        LogInfo(condition=condition, msg=""),
        LogInfo(condition=condition, msg="================================================================"),
        LogInfo(condition=condition, msg=["[VGR GSL HOUSE01] ", title]),
    ]
    if details:
        for detail in details:
            actions.append(LogInfo(condition=condition, msg=["  ", *detail]))
    actions.append(LogInfo(condition=condition, msg="================================================================"))
    return actions


def _read_simple_yaml(yaml_file):
    values = {}
    for raw_line in yaml_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def _read_p2_pgm(pgm_file):
    tokens = []
    for raw_line in pgm_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            tokens.extend(line.split())

    if not tokens or tokens[0] != "P2":
        raise RuntimeError(f"Expected an ASCII P2 PGM map, got '{pgm_file}'.")

    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    data = [int(value) for value in tokens[4:]]
    if len(data) != width * height:
        raise RuntimeError(
            f"PGM '{pgm_file}' has {len(data)} pixels, expected {width * height}."
        )

    return width, height, max_value, data


def _write_p2_pgm(pgm_file, width, height, max_value, data):
    lines = ["P2", f"{width} {height}", str(max_value)]
    for row in range(height):
        start = row * width
        lines.append(" ".join(str(value) for value in data[start:start + width]))
    pgm_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _make_rviz_config_compatible(source_config, target_config):
    text = source_config.read_text(encoding="utf-8")
    # The original PMFS RViz configs reference an optional nav_assistant_tools
    # plugin that is not present in a standard Nav2 RViz install.
    text = text.replace("nav_assistant_tools/SetNavGoal", "rviz_default_plugins/SetGoal")
    target_config.write_text(text, encoding="utf-8")
    return target_config


def _resample_binary_map(width, height, data, source_resolution, target_resolution, free_value, blocked_value):
    if not target_resolution or math.isclose(source_resolution, target_resolution):
        return width, height, data[:]

    map_width_m = width * source_resolution
    map_height_m = height * source_resolution
    target_width = max(1, int(round(map_width_m / target_resolution)))
    target_height = max(1, int(round(map_height_m / target_resolution)))
    resampled = []

    for target_row in range(target_height):
        source_row_start = int(math.floor(target_row * target_resolution / source_resolution))
        source_row_end = int(math.ceil((target_row + 1) * target_resolution / source_resolution))
        source_row_start = min(max(source_row_start, 0), height - 1)
        source_row_end = min(max(source_row_end, source_row_start + 1), height)

        for target_col in range(target_width):
            source_col_start = int(math.floor(target_col * target_resolution / source_resolution))
            source_col_end = int(math.ceil((target_col + 1) * target_resolution / source_resolution))
            source_col_start = min(max(source_col_start, 0), width - 1)
            source_col_end = min(max(source_col_end, source_col_start + 1), width)

            free_cells = 0
            total_cells = 0
            for source_row in range(source_row_start, source_row_end):
                for source_col in range(source_col_start, source_col_end):
                    total_cells += 1
                    if data[source_row * width + source_col] == free_value:
                        free_cells += 1

            # Majority sampling gives a practical in-between map resolution without
            # making House01's already narrow passages vanish completely.
            resampled.append(free_value if free_cells >= total_cells / 2 else blocked_value)

    return target_width, target_height, resampled


def _make_navigation_safe_map(source_yaml, target_yaml, target_resolution=None, close_diagonal_gaps=True):
    metadata = _read_simple_yaml(source_yaml)
    source_image = Path(metadata["image"])
    if not source_image.is_absolute():
        source_image = source_yaml.parent / source_image

    width, height, max_value, data = _read_p2_pgm(source_image)
    source_resolution = float(metadata.get("resolution", "0.1"))
    output_resolution = target_resolution or source_resolution
    free_value = max_value
    blocked_value = 0
    safe_data = data[:]
    blocked_cells = set()

    if close_diagonal_gaps:
        # House01 contains diagonal one-pixel gaps that look passable to a grid planner,
        # but trap the simulated robot/controller. Close only those corner-cutting leaks.
        for row in range(height - 1):
            for col in range(width - 1):
                top_left = row * width + col
                top_right = top_left + 1
                bottom_left = top_left + width
                bottom_right = bottom_left + 1

                a_free = data[top_left] == free_value
                b_free = data[top_right] == free_value
                c_free = data[bottom_left] == free_value
                d_free = data[bottom_right] == free_value

                if a_free and d_free and not b_free and not c_free:
                    blocked_cells.update((top_left, bottom_right))
                if b_free and c_free and not a_free and not d_free:
                    blocked_cells.update((top_right, bottom_left))

        for index in blocked_cells:
            safe_data[index] = blocked_value

        for row in range(height):
            for col in range(width):
                index = row * width + col
                if safe_data[index] != free_value:
                    continue
                neighbors = (
                    (col > 0 and safe_data[index - 1] == free_value),
                    (col + 1 < width and safe_data[index + 1] == free_value),
                    (row > 0 and safe_data[index - width] == free_value),
                    (row + 1 < height and safe_data[index + width] == free_value),
                )
                if not any(neighbors):
                    safe_data[index] = blocked_value

    output_width, output_height, output_data = _resample_binary_map(
        width,
        height,
        safe_data,
        source_resolution,
        output_resolution,
        free_value,
        blocked_value,
    )

    target_image = target_yaml.with_name("navigation_occupancy.pgm")
    _write_p2_pgm(target_image, output_width, output_height, max_value, output_data)
    target_yaml.write_text(
        "\n".join([
            f"image: {target_image.name}",
            f"resolution: {output_resolution:g}",
            f"origin: {metadata.get('origin', '[0, 0, 0]')}",
            f"occupied_thresh: {metadata.get('occupied_thresh', '0.9')}",
            f"free_thresh: {metadata.get('free_thresh', '0.1')}",
            f"negate: {metadata.get('negate', '0')}",
            "",
        ]),
        encoding="utf-8",
    )
    return {
        "blocked_diagonal_cells": len(blocked_cells),
        "source_resolution": source_resolution,
        "output_resolution": output_resolution,
        "source_size": f"{width}x{height}",
        "output_size": f"{output_width}x{output_height}",
    }


def method_defaults(method):
    defaults = {
        "scale": 3,
        "markers_height": 0.2,
        "useDiffusionTerm": True,
        "stdevHit": 1.0,
        "stdevMiss": 1.2,
        "headless": False,
        "distanceWeight": 0.15,
        "openMoveSetExpasion": 5,
        "explorationProbability": 0.05,
        "maxUpdatesPerStop": 5,
        "kernelSigma": 1.5,
        "kernelStretchConstant": 1.5,
        "hitPriorProbability": 0.3,
        "confidenceSigmaSpatial": 1.0,
        "confidenceMeasurementWeight": 1.0,
        "useWindGroundTruth": True,
        "stepsSourceUpdate": 3,
        "maxRegionSize": 5,
        "refineFraction": 0.1,
        "blurSigmaX": 1.5,
        "blurSigmaY": 1.5,
        "step": 0.5,
        "initial_step": 0.6,
        "step_increment": 0.3,
        "Kmu": 0.5,
        "Kp": 1.0,
        "intervalLength": 0.5,
        "initSpiralStep": 1.0,
        "spiralStep_increment": 0.4,
        "numberOfParticles": 500,
        "maxEstimations": 20,
        "numberOfWindObs": 30,
        "convergenceThr": 0.5,
        "deltaT": 1.0,
        "mu": 0.9,
        "Sp": 0.01,
        "Rconv": 0.5,
        "use_pmfs_rviz": method in PMFS_STYLE_METHODS,
    }

    if method == "GrGSL":
        defaults["step"] = 0.7
    elif method == "SurgeCast":
        defaults["step"] = 1.0
        defaults["use_pmfs_rviz"] = False
    elif method == "SurgeSpiral":
        defaults["step"] = 1.0
        defaults["initSpiralStep"] = 1.0
        defaults["spiralStep_increment"] = 0.4
        defaults["use_pmfs_rviz"] = False
    elif method == "Spiral":
        defaults["initial_step"] = 0.6
        defaults["step_increment"] = 0.3
        defaults["intervalLength"] = 0.5
        defaults["use_pmfs_rviz"] = False
    elif method == "ParticleFilter":
        defaults["step"] = 1.0
        defaults["numberOfParticles"] = 500
        defaults["convergenceThr"] = 0.5
        defaults["use_pmfs_rviz"] = False

    return defaults


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="House01"),
        DeclareLaunchArgument("simulation", default_value="1,3-2,4_fast"),
        DeclareLaunchArgument("method", default_value="PMFS"),
        DeclareLaunchArgument("use_infotaxis", default_value="True"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("use_hit_rviz", default_value="True"),
        DeclareLaunchArgument("use_source_rviz", default_value="True"),
        DeclareLaunchArgument("gsl_scale", default_value="1"),
        DeclareLaunchArgument("gsl_map_resolution", default_value="0.16"),
        DeclareLaunchArgument("convergence_thr", default_value=""),
        DeclareLaunchArgument("robot_radius", default_value="0.02"),
        DeclareLaunchArgument("close_diagonal_gaps", default_value="True"),
        DeclareLaunchArgument("basic_sim_log_level", default_value="WARN"),
    ]


def launch_setup(context, *args, **kwargs):
    read_sim_yaml(context)

    robot_name = LaunchConfiguration("robot_name")
    pkg_dir = get_package_share_directory("vgr_pmfs_house01_env")
    pmfs_launch_dir = Path(get_package_share_directory("pmfs_env")) / "launch"
    vgr_dir = get_package_share_directory("vgr_dataset")
    scenario = LaunchConfiguration("scenario").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    method = LaunchConfiguration("method").perform(context)
    gsl_scale = int(LaunchConfiguration("gsl_scale").perform(context))
    gsl_map_resolution_value = LaunchConfiguration("gsl_map_resolution").perform(context).strip()
    gsl_map_resolution = float(gsl_map_resolution_value) if gsl_map_resolution_value else None
    convergence_thr = LaunchConfiguration("convergence_thr").perform(context).strip()
    robot_radius = LaunchConfiguration("robot_radius").perform(context)
    close_diagonal_gaps = _truthy(LaunchConfiguration("close_diagonal_gaps").perform(context))
    start_pos_x = LaunchConfiguration("start_pos_x").perform(context)
    start_pos_y = LaunchConfiguration("start_pos_y").perform(context)
    start_pos_z = LaunchConfiguration("start_pos_z").perform(context)

    if method in SEMANTIC_METHODS:
        raise RuntimeError(
            f"Method '{method}' requires the semantics stack and extra ontology/detection inputs. "
            "The House01 VGR wrapper currently supports non-semantic methods only."
        )
    if method not in NON_SEMANTIC_METHODS:
        raise RuntimeError(
            f"Unsupported method '{method}'. Supported methods for this wrapper are: "
            + ", ".join(sorted(NON_SEMANTIC_METHODS))
        )
    if gsl_scale < 1 or gsl_scale > 5:
        raise RuntimeError(
            f"House01 is a small 0.1 m/cell map, so gsl_scale must be between 1 and 5. "
            f"Got gsl_scale={gsl_scale}. Large PMFS scales collapse the map to zero free cells."
        )
    if gsl_map_resolution is not None and gsl_map_resolution <= 0:
        raise RuntimeError(
            f"gsl_map_resolution must be positive when provided. Got {gsl_map_resolution}."
        )

    defaults = method_defaults(method)
    convergence_thr_display = convergence_thr if convergence_thr else "algorithm default"
    results_dir = Path("results") / method
    results_dir.mkdir(parents=True, exist_ok=True)
    results_file = results_dir / f"House01_{simulation}.csv"

    runtime_dir = Path(tempfile.gettempdir()) / "vgr_pmfs_house01_env" / scenario / simulation
    runtime_dir.mkdir(parents=True, exist_ok=True)
    rviz_hit_config = _make_rviz_config_compatible(
        pmfs_launch_dir / "hit.rviz",
        runtime_dir / "hit_compatible.rviz",
    )
    rviz_source_config = _make_rviz_config_compatible(
        pmfs_launch_dir / "source.rviz",
        runtime_dir / "source_compatible.rviz",
    )
    source_map_file = Path(vgr_dir) / "scenarios" / scenario / "occupancy.yaml"
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

    world_file = runtime_dir / "basic_sim_house01.yaml"
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
        {"stop_and_measure_time": 0.4},
        {"th_gas_present": parse_substitution("$(var th_gas_present)")},
        {"th_wind_present": parse_substitution("$(var th_wind_present)")},
        {"ground_truth_x": parse_substitution("$(var source_x)")},
        {"ground_truth_y": parse_substitution("$(var source_y)")},
        {"resultsFile": parse_substitution("results/$(var method)/House01_$(var simulation).csv")},
        {"scale": parse_substitution("$(var gsl_scale)")},
        {"markers_height": defaults["markers_height"]},
        {"anemometer_frame": parse_substitution("$(var robot_name)_anemometer_frame")},
        {"openMoveSetExpasion": defaults["openMoveSetExpasion"]},
        {"explorationProbability": defaults["explorationProbability"]},
        {"useDiffusionTerm": defaults["useDiffusionTerm"]},
        {"stdevHit": defaults["stdevHit"]},
        {"stdevMiss": defaults["stdevMiss"]},
        {"infoTaxis": parse_substitution("$(var use_infotaxis)")},
        {"allowMovementRepetition": parse_substitution("$(var use_infotaxis)")},
        {"headless": defaults["headless"]},
        {"distanceWeight": defaults["distanceWeight"]},
        {"maxUpdatesPerStop": defaults["maxUpdatesPerStop"]},
        {"kernelSigma": defaults["kernelSigma"]},
        {"kernelStretchConstant": defaults["kernelStretchConstant"]},
        {"hitPriorProbability": defaults["hitPriorProbability"]},
        {"confidenceSigmaSpatial": defaults["confidenceSigmaSpatial"]},
        {"confidenceMeasurementWeight": defaults["confidenceMeasurementWeight"]},
        {"initialExplorationMoves": parse_substitution("$(var initialExplorationMoves)")},
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
        ],
    )

    delayed_sensor_and_server_nodes = TimerAction(
        period=3.0,
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
        period=9.0,
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
                ],
            ),
            *gsl_call,
        ],
    )

    actions = []
    actions.extend(loud_logs(
        "PREPARING HOUSE01 GSL WRAPPER",
        details=[
            ["scenario:", LaunchConfiguration("scenario")],
            ["simulation:", LaunchConfiguration("simulation")],
            ["method:", LaunchConfiguration("method")],
            ["robot:", robot_name],
            ["runtime dir:", str(runtime_dir)],
            ["results file:", str(results_file)],
            ["world file:", str(world_file)],
            ["navigation map:", str(navigation_map_file)],
            ["hit rviz config:", str(rviz_hit_config)],
            ["source rviz config:", str(rviz_source_config)],
            ["map resolution:", f"{map_stats['source_resolution']:g}", " -> ", f"{map_stats['output_resolution']:g}"],
            ["map size:", map_stats["source_size"], " -> ", map_stats["output_size"]],
            ["effective GSL cell:", f"{effective_gsl_cell_size:g} m"],
            ["diagonal cells blocked:", str(map_stats["blocked_diagonal_cells"])],
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
            ["robot_radius:", LaunchConfiguration("robot_radius")],
            ["close_diagonal_gaps:", LaunchConfiguration("close_diagonal_gaps")],
            ["basic_sim_log_level:", LaunchConfiguration("basic_sim_log_level")],
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
    if LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1", "yes") and defaults["use_pmfs_rviz"]:
        actions.extend(loud_logs(
            "OPENING HIT RVIZ WINDOW",
            details=[
                ["config:", str(pmfs_launch_dir / "hit.rviz")],
                ["runtime config:", str(rviz_hit_config)],
            ],
            condition=hit_rviz_condition,
        ))
        actions.append(rviz_hit_log)
        actions.append(rviz_hit)
        actions.extend(loud_logs(
            "OPENING SOURCE RVIZ WINDOW",
            details=[
                ["config:", str(pmfs_launch_dir / "source.rviz")],
                ["runtime config:", str(rviz_source_config)],
            ],
            condition=source_rviz_condition,
        ))
        actions.append(rviz_source_log)
        actions.append(rviz_source)
    elif LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1", "yes"):
        actions.extend(loud_logs(
            "METHOD DOES NOT USE PMFS HIT/SOURCE RVIZ",
            details=[
                ["method:", method],
                ["reason:", "this wrapper only reuses hit.rviz/source.rviz for PMFS"],
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
        SetLaunchConfiguration(name="th_gas_present", value="0.1"),
        SetLaunchConfiguration(name="th_wind_present", value="0.02"),
        SetLaunchConfiguration(name="player_freq", value="0.1"),
        SetLaunchConfiguration(name="filament_movement_stdev", value="0.5"),
        SetLaunchConfiguration(name="sourceDiscriminationPower", value="0.3"),
        SetLaunchConfiguration(name="iterationsToRecord", value="200"),
        SetLaunchConfiguration(name="minWarmupIterations", value="0"),
        SetLaunchConfiguration(name="maxWarmupIterations", value="500"),
        SetLaunchConfiguration(name="initialExplorationMoves", value="2"),
        SetLaunchConfiguration(name="filamentDeltaTime", value="0.1"),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description)
