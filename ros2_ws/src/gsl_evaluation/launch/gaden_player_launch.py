import os
import shutil
import struct
import tempfile
import zlib
from array import array
from csv import reader
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

import sys
sys.path.append(get_package_share_directory("gaden_common"))
from gaden_internal_py.utils import read_sim_yaml  # NOQA # type: ignore


GADEN_VERSION_MAJOR = 3
GADEN_VERSION_MINOR = 0
GADEN_RESULT_IDENTIFIER = b"GADEN_RESULT\x00"
GADEN_COMPRESSION_ZLIB = 1


def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="House01"),
        DeclareLaunchArgument("simulation", default_value="1,3-2,4_fast"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
    ]


def _read_occupancy_metadata(occupancy_file: Path):
    min_coord = None
    max_coord = None
    dimensions = None
    cell_size = None

    with occupancy_file.open("r", encoding="utf-8") as handle:
        for _ in range(4):
            line = handle.readline().strip()
            if line.startswith("#env_min(m) "):
                _, x, y, z = line.split()
                min_coord = (float(x), float(y), float(z))
            elif line.startswith("#env_max(m) "):
                _, x, y, z = line.split()
                max_coord = (float(x), float(y), float(z))
            elif line.startswith("#num_cells "):
                _, x, y, z = line.split()
                dimensions = (int(x), int(y), int(z))
            elif line.startswith("#cell_size(m) "):
                _, value = line.split()
                cell_size = float(value)

    if min_coord is None or max_coord is None or dimensions is None or cell_size is None:
        raise RuntimeError(f"Could not parse occupancy metadata from '{occupancy_file}'.")

    return min_coord, max_coord, dimensions, cell_size


def _wind_csv_files(wind_source: Path, simulation: str):
    files = []
    prefix = f"{simulation}_"
    for candidate in wind_source.glob(f"{simulation}_*.csv"):
        name = candidate.name
        if name.endswith((".csv_U", ".csv_V", ".csv_W")):
            continue
        suffix = name[len(prefix):-4]
        if suffix.isdigit():
            files.append((int(suffix), candidate))

    files.sort(key=lambda item: item[0])
    return [path for _, path in files]


def _converted_wind_is_current(wind_target: Path, csv_files):
    if not wind_target.is_dir():
        return False

    converted = sorted(wind_target.glob("wind_iteration_*"))
    if len(converted) != len(csv_files):
        return False

    for index, source_file in enumerate(csv_files):
        target_file = wind_target / f"wind_iteration_{index}"
        if not target_file.is_file():
            return False
        if target_file.stat().st_mtime < source_file.stat().st_mtime:
            return False

    return True


def _gas_iteration_files(gas_source: Path):
    files = []
    for candidate in gas_source.glob("iteration_*"):
        suffix = candidate.name.split("_", 1)[-1]
        if suffix.isdigit():
            files.append((int(suffix), candidate))

    files.sort(key=lambda item: item[0])
    return [path for _, path in files]


def _patched_gas_is_current(gas_target: Path, iteration_files):
    if gas_target.is_symlink() or not gas_target.is_dir():
        return False

    converted = sorted(gas_target.glob("iteration_*"))
    if len(converted) != len(iteration_files):
        return False

    for source_file in iteration_files:
        target_file = gas_target / source_file.name
        if not target_file.is_file():
            return False
        if target_file.stat().st_mtime < source_file.stat().st_mtime:
            return False

    return True


def _read_gas_payload(source_file: Path):
    data = source_file.read_bytes()

    if data.startswith(GADEN_RESULT_IDENTIFIER):
        mode_offset = len(GADEN_RESULT_IDENTIFIER)
        size_offset = mode_offset + 1
        payload_offset = size_offset + 8

        mode = data[mode_offset]
        uncompressed_size = struct.unpack_from("<Q", data, size_offset)[0]
        payload = data[payload_offset:]

        if mode != GADEN_COMPRESSION_ZLIB:
            return data, None, True

        raw = bytearray(zlib.decompress(payload))
        if uncompressed_size and len(raw) != uncompressed_size:
            raise RuntimeError(
                f"Unexpected decompressed size for '{source_file}': "
                f"expected {uncompressed_size}, got {len(raw)}."
            )

        return data, raw, True

    return data, bytearray(zlib.decompress(data)), False


def _write_gas_payload(target_file: Path, raw: bytes, modern_header: bool):
    compressed = zlib.compress(bytes(raw), level=1)
    if modern_header:
        target_file.write_bytes(
            GADEN_RESULT_IDENTIFIER
            + bytes([GADEN_COMPRESSION_ZLIB])
            + struct.pack("<Q", len(raw))
            + compressed
        )
    else:
        target_file.write_bytes(compressed)


def _convert_legacy_gas_logs(gas_source: Path, gas_target: Path, occupancy_file: Path):
    iteration_files = _gas_iteration_files(gas_source)
    if not iteration_files:
        raise RuntimeError(f"No gas iteration files were found in '{gas_source}'.")

    if _patched_gas_is_current(gas_target, iteration_files):
        return

    if gas_target.exists() or gas_target.is_symlink():
        if gas_target.is_symlink() or gas_target.is_file():
            gas_target.unlink()
        else:
            shutil.rmtree(gas_target)
    gas_target.mkdir(parents=True, exist_ok=True)

    min_coord, max_coord, dimensions, cell_size = _read_occupancy_metadata(occupancy_file)

    for source_file in iteration_files:
        original_data, raw, modern_header = _read_gas_payload(source_file)
        target_file = gas_target / source_file.name

        if raw is None:
            target_file.write_bytes(original_data)
            continue

        version_major = struct.unpack_from("<i", raw, 0)[0]
        if version_major != 1:
            target_file.write_bytes(original_data)
            continue

        # VGR House01 logs can use a legacy header layout that current GADEN misparses.
        # Rewrite the environment section in the exact format expected by the v1 loader.
        struct.pack_into("<3d", raw, 4, *min_coord)
        struct.pack_into("<3d", raw, 28, *max_coord)
        struct.pack_into("<3i", raw, 52, *dimensions)
        struct.pack_into("<3d", raw, 64, cell_size, cell_size, cell_size)

        _write_gas_payload(target_file, raw, modern_header)


def _convert_wind_files(wind_source: Path, wind_target: Path, occupancy_file: Path, simulation: str):
    csv_files = _wind_csv_files(wind_source, simulation)
    if not csv_files:
        raise RuntimeError(f"No supported wind CSV files were found in '{wind_source}'.")

    if _converted_wind_is_current(wind_target, csv_files):
        return

    if wind_target.exists() or wind_target.is_symlink():
        if wind_target.is_symlink() or wind_target.is_file():
            wind_target.unlink()
        else:
            shutil.rmtree(wind_target)
    wind_target.mkdir(parents=True, exist_ok=True)

    min_coord, _, dimensions, cell_size = _read_occupancy_metadata(occupancy_file)
    num_cells = dimensions[0] * dimensions[1] * dimensions[2]

    for index, csv_file in enumerate(csv_files):
        wind_map = array("f", [0.0]) * (num_cells * 3)

        with csv_file.open("r", encoding="utf-8", newline="") as handle:
            csv_reader = reader(handle)
            header = next(csv_reader, None)
            if not header:
                raise RuntimeError(f"Wind CSV '{csv_file}' is empty.")

            points_first = header[0].startswith("Points")
            for row in csv_reader:
                if len(row) < 6:
                    continue

                if points_first:
                    px, py, pz = (float(value) for value in row[0:3])
                    vx, vy, vz = (float(value) for value in row[3:6])
                else:
                    vx, vy, vz = (float(value) for value in row[0:3])
                    px, py, pz = (float(value) for value in row[3:6])

                ix = int((px - min_coord[0]) / cell_size)
                iy = int((py - min_coord[1]) / cell_size)
                iz = int((pz - min_coord[2]) / cell_size)

                if not (0 <= ix < dimensions[0] and 0 <= iy < dimensions[1] and 0 <= iz < dimensions[2]):
                    continue

                cell_index = ix + iy * dimensions[0] + iz * dimensions[0] * dimensions[1]
                data_index = cell_index * 3
                wind_map[data_index] = vx
                wind_map[data_index + 1] = vy
                wind_map[data_index + 2] = vz

        output_file = wind_target / f"wind_iteration_{index}"
        with output_file.open("wb") as handle:
            handle.write(struct.pack("<ii", GADEN_VERSION_MAJOR, GADEN_VERSION_MINOR))
            wind_map.tofile(handle)


def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    pkg_dir = LaunchConfiguration("pkg_dir").perform(context)
    vgr_dir = get_package_share_directory("vgr_dataset")

    params_yaml_file = os.path.join(pkg_dir, "scenarios", scenario, "params", "gaden_params.yaml")

    read_sim_yaml(context)

    source_x = LaunchConfiguration("source_x").perform(context)
    source_y = LaunchConfiguration("source_y").perform(context)
    source_z = LaunchConfiguration("source_z").perform(context)
    gas_type = LaunchConfiguration("gas_type").perform(context)

    runtime_dir = Path(tempfile.gettempdir()) / "vgr_pmfs_house01_env" / scenario / simulation
    gas_target = runtime_dir / "gas_simulations" / simulation / f"FilamentSimulation_gasType_{gas_type}_sourcePosition_{source_x}_{source_y}_{source_z}"
    gas_source = Path(vgr_dir) / "scenarios" / scenario / "gas_simulations" / simulation / gas_target.name
    wind_target = runtime_dir / "wind"
    wind_source = Path(vgr_dir) / "scenarios" / scenario / "wind_simulations" / simulation
    occupancy_file = Path(vgr_dir) / "scenarios" / scenario / "OccupancyGrid3D.csv"

    runtime_dir.mkdir(parents=True, exist_ok=True)
    gas_target.parent.mkdir(parents=True, exist_ok=True)

    _convert_legacy_gas_logs(gas_source, gas_target, occupancy_file)
    _convert_wind_files(wind_source, wind_target, occupancy_file, simulation)

    SetLaunchConfiguration(name="runtime_dir", value=str(runtime_dir)).execute(context)

    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("vgr_dataset"),
                    "scenarios",
                    scenario,
                    "launch",
                    "gaden_ros2.rviz",
                ),
            ],
        ),
        Node(
            package="gaden_environment",
            executable="environment",
            name="gaden_environment",
            output="screen",
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)],
        ),
        Node(
            package="gaden_player",
            executable="player",
            name="gaden_player",
            output="screen",
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)],
        ),
    ]


def generate_launch_description():
    launch_description = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("vgr_pmfs_house01_env")],
        ),
    ]

    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_description)
