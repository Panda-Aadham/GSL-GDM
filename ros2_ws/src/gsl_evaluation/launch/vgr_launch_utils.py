"""Shared helpers for VGR dataset launch wrappers."""

import re

from launch.actions import SetLaunchConfiguration


def clean_scalar(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def read_simple_yaml(yaml_file):
    values = {}
    for raw_line in yaml_file.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = clean_scalar(value)
    return values


def parse_origin(origin_value):
    return [
        float(value)
        for value in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", origin_value)
    ]


def parse_vgr_simulation_launch(launch_file):
    text = launch_file.read_text(encoding="utf-8", errors="ignore")
    args = dict(re.findall(r'<arg\s+name="([^"]+)"\s+default="([^"]*)"', text))

    required = ("source_location_x", "source_location_y", "source_location_z", "gas_type")
    missing = [name for name in required if name not in args]
    if missing:
        raise RuntimeError(f"Could not parse {', '.join(missing)} from '{launch_file}'.")

    return {
        "source_x": args["source_location_x"],
        "source_y": args["source_location_y"],
        "source_z": args["source_location_z"],
        "gas_type": args["gas_type"],
        "initial_iteration": 280,
        "allow_looping": "true",
        "loop_from_iteration": 280,
        "loop_to_iteration": 288,
    }


def set_launch_configs(context, values):
    for key, value in values.items():
        SetLaunchConfiguration(name=key, value=str(value)).execute(context)


def next_csv_run_index(csv_file, index_column="run_index"):
    """Return one greater than the largest run index in an existing CSV file."""
    if not csv_file.is_file() or csv_file.stat().st_size == 0:
        return 1

    lines = csv_file.read_text(encoding="utf-8", errors="ignore").splitlines()
    header = None
    header_index = None
    max_index = 0

    for line in lines:
        if not line.strip():
            continue
        columns = [column.strip() for column in line.split(",")]
        if index_column in columns:
            header = columns
            header_index = header.index(index_column)
            continue
        if header is None or header_index is None or len(columns) <= header_index:
            continue
        try:
            max_index = max(max_index, int(float(columns[header_index])))
        except ValueError:
            continue

    return max_index + 1
