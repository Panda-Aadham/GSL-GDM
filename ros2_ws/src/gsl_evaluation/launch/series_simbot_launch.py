from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


PASSTHROUGH_ARGUMENTS = [
    "scenario",
    "simulation",
    "method",
    "use_infotaxis",
    "stop_and_measure_time",
    "th_gas_present",
    "th_wind_present",
    "max_wait_for_gas_time",
    "global_exploration_on_gas_timeout",
    "grgsl_global_move_fallback",
    "use_rviz",
    "use_hit_rviz",
    "use_source_rviz",
    "gsl_scale",
    "gsl_map_resolution",
    "convergence_thr",
    "robot_radius",
    "close_diagonal_gaps",
    "basic_sim_log_level",
    "rviz_log_level",
    "gsl_start_delay",
    "gsl_call_delay",
    "dynamic_threshold_target_error",
    "dynamic_threshold_alpha",
    "dynamic_threshold_max_error",
    "variance_log_interval",
]


def _launch_arguments():
    return [
        DeclareLaunchArgument("runs", default_value="5"),
        DeclareLaunchArgument("scenario_set", default_value="single"),
        DeclareLaunchArgument("simulation_speed", default_value="fast"),
        DeclareLaunchArgument("houses", default_value=""),
        DeclareLaunchArgument("house_start", default_value="1"),
        DeclareLaunchArgument("house_end", default_value="30"),
        DeclareLaunchArgument("simulations_csv", default_value=""),
        DeclareLaunchArgument("start_run_index", default_value="auto"),
        DeclareLaunchArgument("inter_run_delay", default_value="2.0"),
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
        DeclareLaunchArgument("use_rviz", default_value="False"),
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
    ]


def _house_number(house_name):
    if not house_name.startswith("House"):
        raise RuntimeError(f"Expected house names like House01. Got {house_name}.")
    return int(house_name[5:])


def _parse_requested_houses(value):
    value = value.strip()
    if not value:
        return None

    houses = set()
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            start, end = token.split("-", 1)
            start_number = _house_number(start) if start.startswith("House") else int(start)
            end_number = _house_number(end) if end.startswith("House") else int(end)
            for number in range(start_number, end_number + 1):
                houses.add(f"House{number:02d}")
        elif token.startswith("House"):
            houses.add(token)
        else:
            houses.add(f"House{int(token):02d}")

    return houses


def _read_simulation_groups(csv_file):
    groups = [[]]
    for raw_line in csv_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            if groups[-1]:
                groups.append([])
            continue

        columns = [column.strip() for column in line.split(";")]
        if len(columns) < 2:
            continue
        groups[-1].append((columns[0], columns[1]))

    return [group for group in groups if group]


def _launch_cli_arguments(arguments):
    return [
        f"{name}:={value}"
        for name, value in arguments.items()
        if str(value).strip() != ""
    ]


def _default_simulations_csv():
    vgr_share = Path(get_package_share_directory("vgr_dataset"))
    candidates = [
        vgr_share / "simulations.csv",
        (vgr_share / "scenarios").resolve().parent.parent / "simulations.csv",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def _expand_dataset_scenarios(context):
    scenario_set = LaunchConfiguration("scenario_set").perform(context).strip().lower()
    simulation_speed = LaunchConfiguration("simulation_speed").perform(context).strip().lower()

    if scenario_set in ("", "single"):
        return [(
            LaunchConfiguration("scenario").perform(context),
            LaunchConfiguration("simulation").perform(context),
        )]

    if scenario_set not in ("first", "second", "all"):
        raise RuntimeError("scenario_set must be one of: single, first, second, all.")
    if simulation_speed not in ("fast", "slow", "both"):
        raise RuntimeError("simulation_speed must be one of: fast, slow, both.")

    csv_arg = LaunchConfiguration("simulations_csv").perform(context).strip()
    if csv_arg:
        csv_file = Path(csv_arg)
    else:
        csv_file = _default_simulations_csv()
    if not csv_file.is_file():
        raise RuntimeError(f"Could not find simulations CSV at '{csv_file}'.")

    groups = _read_simulation_groups(csv_file)
    if len(groups) < 2 and scenario_set in ("second", "all"):
        raise RuntimeError(f"Expected two simulation groups in '{csv_file}', but found {len(groups)}.")

    if scenario_set == "first":
        selected_groups = [groups[0]]
    elif scenario_set == "second":
        selected_groups = [groups[1]]
    else:
        selected_groups = groups[:2]

    requested_houses = _parse_requested_houses(LaunchConfiguration("houses").perform(context))
    house_start = int(LaunchConfiguration("house_start").perform(context))
    house_end = int(LaunchConfiguration("house_end").perform(context))
    if house_start < 1 or house_end < house_start:
        raise RuntimeError(f"Invalid house range: house_start={house_start}, house_end={house_end}.")

    speeds = ["fast", "slow"] if simulation_speed == "both" else [simulation_speed]
    tasks = []
    for group in selected_groups:
        for scenario, base_simulation in group:
            house_num = _house_number(scenario)
            if requested_houses is not None and scenario not in requested_houses:
                continue
            if house_num < house_start or house_num > house_end:
                continue
            for speed in speeds:
                tasks.append((scenario, f"{base_simulation}_{speed}"))

    if not tasks:
        raise RuntimeError("Dataset expansion produced no runs. Check scenario_set, houses, and house range arguments.")

    return tasks


def _series_setup(context, *args, **kwargs):
    runs = int(LaunchConfiguration("runs").perform(context))
    if runs < 1:
        raise RuntimeError(f"runs must be a positive integer. Got {runs}.")

    inter_run_delay = float(LaunchConfiguration("inter_run_delay").perform(context))
    if inter_run_delay < 0:
        raise RuntimeError(f"inter_run_delay must be non-negative. Got {inter_run_delay}.")

    start_run_index = LaunchConfiguration("start_run_index").perform(context).strip()
    auto_run_index = start_run_index.lower() in ("", "auto")
    if not auto_run_index and int(start_run_index) < 1:
        raise RuntimeError(f"start_run_index must be a positive integer or 'auto'. Got {start_run_index}.")

    method = LaunchConfiguration("method").perform(context)
    scenario_tasks = _expand_dataset_scenarios(context)

    passthrough = {
        name: LaunchConfiguration(name).perform(context)
        for name in PASSTHROUGH_ARGUMENTS
    }

    run_processes = []
    run_descriptions = []
    total_runs = len(scenario_tasks) * runs
    for scenario, simulation in scenario_tasks:
        scenario_passthrough = passthrough.copy()
        scenario_passthrough["scenario"] = scenario
        scenario_passthrough["simulation"] = simulation

        for run_offset in range(runs):
            run_number = len(run_processes) + 1
            if auto_run_index:
                run_index = "auto"
            else:
                run_index = str(int(start_run_index) + run_offset)

            command = [
                "ros2",
                "launch",
                "vgr_pmfs_house01_env",
                "main_simbot_launch.py",
                *_launch_cli_arguments(scenario_passthrough),
                f"run_index:={run_index}",
            ]

            run_descriptions.append((scenario, simulation, run_index))
            run_processes.append(
                ExecuteProcess(
                    cmd=command,
                    name=f"vgr_gsl_series_run_{run_number}",
                    output="screen",
                )
            )

    actions = [
        LogInfo(msg=""),
        LogInfo(msg="================================================================"),
        LogInfo(msg="[VGR GSL SERIES] PREPARING SEQUENTIAL RUNS"),
        LogInfo(msg=f"  scenario_set:{LaunchConfiguration('scenario_set').perform(context)}"),
        LogInfo(msg=f"  simulation_speed:{LaunchConfiguration('simulation_speed').perform(context)}"),
        LogInfo(msg=f"  scenario tasks:{len(scenario_tasks)}"),
        LogInfo(msg=f"  method:{method}"),
        LogInfo(msg=f"  runs per scenario:{runs}"),
        LogInfo(msg=f"  total child launches:{total_runs}"),
        LogInfo(msg=f"  start_run_index:{start_run_index}"),
        LogInfo(msg=f"  inter_run_delay:{inter_run_delay:g} s"),
        LogInfo(msg="  note:each run starts only after the previous child launch exits"),
        LogInfo(msg="================================================================"),
    ]

    for index, process in enumerate(run_processes[1:], start=1):
        run_number = index + 1
        previous_process = run_processes[index - 1]
        scenario, simulation, run_index = run_descriptions[index]
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=previous_process,
                    on_exit=[
                        LogInfo(msg=""),
                        LogInfo(
                            msg=(
                                f"[VGR GSL SERIES] PREVIOUS RUN FINISHED; STARTING RUN "
                                f"{run_number}/{total_runs}: {scenario} {simulation} run_index={run_index}"
                            )
                        ),
                        TimerAction(period=inter_run_delay, actions=[process]),
                    ],
                )
            )
        )

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=run_processes[-1],
                on_exit=[
                    LogInfo(msg=""),
                    LogInfo(msg="[VGR GSL SERIES] ALL REQUESTED RUNS FINISHED"),
                ],
            )
        )
    )
    actions.extend([
        LogInfo(msg=""),
        LogInfo(
            msg=(
                f"[VGR GSL SERIES] STARTING RUN 1/{total_runs}: "
                f"{run_descriptions[0][0]} {run_descriptions[0][1]} run_index={run_descriptions[0][2]}"
            )
        ),
        run_processes[0],
    ])

    return actions


def generate_launch_description():
    launch_description = _launch_arguments()
    launch_description.append(OpaqueFunction(function=_series_setup))
    return LaunchDescription(launch_description)
