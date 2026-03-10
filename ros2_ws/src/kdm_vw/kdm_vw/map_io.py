from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np


@dataclass
class OccupancyMapMetadata:
    image: Path
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    max_value: int
    data: list

    @property
    def size(self):
        return (self.width * self.resolution, self.height * self.resolution)

    @property
    def offset(self):
        return (self.origin_x, self.origin_y)

    @property
    def free_mask(self):
        matrix = np.array(self.data, dtype=int).reshape(self.height, self.width)
        return matrix == self.max_value


def compute_wall_outline_mask(free_mask):
    free_mask = np.asarray(free_mask, dtype=bool)
    obstacle_mask = ~free_mask
    padded_free = np.pad(free_mask, 1, mode="constant", constant_values=False)

    adjacent_to_free = (
        padded_free[1:-1, :-2]
        | padded_free[1:-1, 2:]
        | padded_free[:-2, 1:-1]
        | padded_free[2:, 1:-1]
        | padded_free[:-2, :-2]
        | padded_free[:-2, 2:]
        | padded_free[2:, :-2]
        | padded_free[2:, 2:]
    )
    return obstacle_mask & adjacent_to_free


def clean_scalar(value):
    value = str(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def read_simple_yaml(yaml_file):
    values = {}
    for raw_line in Path(yaml_file).read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = clean_scalar(value)
    return values


def parse_origin(origin_value):
    return [
        float(value)
        for value in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(origin_value))
    ]


def read_p2_pgm(pgm_path):
    tokens = []
    for raw_line in Path(pgm_path).read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            tokens.extend(line.split())

    if not tokens or tokens[0] != "P2":
        raise RuntimeError(f"Expected ASCII P2 PGM map, got '{pgm_path}'.")

    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    data = [int(value) for value in tokens[4:]]
    if len(data) != width * height:
        raise RuntimeError(f"PGM '{pgm_path}' has {len(data)} pixels, expected {width * height}.")

    return width, height, max_value, data


def load_occupancy_map(yaml_path):
    yaml_path = Path(yaml_path)
    metadata = read_simple_yaml(yaml_path)
    image = Path(metadata["image"])
    if not image.is_absolute():
        image = yaml_path.parent / image

    width, height, max_value, data = read_p2_pgm(image)
    origin = parse_origin(metadata.get("origin", "[0, 0, 0]"))
    origin_x = origin[0] if len(origin) > 0 else 0.0
    origin_y = origin[1] if len(origin) > 1 else 0.0

    return OccupancyMapMetadata(
        image=image,
        resolution=float(metadata.get("resolution", "0.1")),
        origin_x=origin_x,
        origin_y=origin_y,
        width=width,
        height=height,
        max_value=max_value,
        data=data,
    )
