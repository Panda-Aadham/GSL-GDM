"""Map processing helpers for the VGR GSL wrapper."""

import importlib.util
import math
from collections import deque
from pathlib import Path


def _load_launch_helper(module_name):
    module_path = Path(__file__).with_name(f"{module_name}.py")
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_launch_utils = _load_launch_helper("vgr_launch_utils")
_parse_origin = _launch_utils.parse_origin
_read_simple_yaml = _launch_utils.read_simple_yaml


def read_p2_pgm(pgm_file):
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


def write_p2_pgm(pgm_file, width, height, max_value, data):
    lines = ["P2", f"{width} {height}", str(max_value)]
    for row in range(height):
        start = row * width
        lines.append(" ".join(str(value) for value in data[start:start + width]))
    pgm_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def auto_start_position(map_yaml, source_x, source_y, min_clearance=0.0):
    metadata = _read_simple_yaml(map_yaml)
    image = Path(metadata["image"])
    if not image.is_absolute():
        image = map_yaml.parent / image

    width, height, max_value, data = read_p2_pgm(image)
    resolution = float(metadata.get("resolution", "0.1"))
    origin = _parse_origin(metadata.get("origin", "[0, 0, 0]"))
    origin_x = origin[0] if len(origin) > 0 else 0.0
    origin_y = origin[1] if len(origin) > 1 else 0.0
    free_value = max_value

    free_cells = {
        (col, row)
        for row in range(height)
        for col in range(width)
        if data[row * width + col] == free_value
    }
    if not free_cells:
        raise RuntimeError(f"No free cells found in '{image}'.")

    obstacle_distance = {cell: 10**9 for cell in free_cells}
    queue = deque()
    for cell in free_cells:
        col, row = cell
        if col == 0 or row == 0 or col == width - 1 or row == height - 1:
            obstacle_distance[cell] = 0
            queue.append(cell)
            continue
        for d_col, d_row in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (col + d_col, row + d_row)
            if neighbor not in free_cells:
                obstacle_distance[cell] = 0
                queue.append(cell)
                break

    while queue:
        col, row = queue.popleft()
        distance = obstacle_distance[(col, row)] + 1
        for d_col, d_row in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (col + d_col, row + d_row)
            if neighbor in obstacle_distance and distance < obstacle_distance[neighbor]:
                obstacle_distance[neighbor] = distance
                queue.append(neighbor)

    components = []
    remaining = set(free_cells)
    while remaining:
        seed = remaining.pop()
        component = {seed}
        queue = deque([seed])
        while queue:
            col, row = queue.popleft()
            for d_col, d_row in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (col + d_col, row + d_row)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)

    largest_component = max(components, key=len)
    source = (float(source_x), float(source_y))

    def cell_to_world(cell):
        col, row = cell
        return (
            origin_x + (col + 0.5) * resolution,
            origin_y + (height - row - 0.5) * resolution,
        )

    min_clearance_cells = max(0, int(math.ceil(min_clearance / resolution)))
    start_candidates = [
        cell for cell in largest_component
        if obstacle_distance.get(cell, 0) >= min_clearance_cells
    ]
    if not start_candidates:
        start_candidates = list(largest_component)

    distances = {
        cell: (cell_to_world(cell)[0] - source[0]) ** 2
        + (cell_to_world(cell)[1] - source[1]) ** 2
        for cell in start_candidates
    }
    max_distance = max(distances.values()) if distances else 0.0
    far_candidates = [
        cell for cell in start_candidates
        if max_distance <= 0 or distances[cell] >= max_distance * 0.6
    ]
    if not far_candidates:
        far_candidates = start_candidates

    max_clearance = max(obstacle_distance.get(cell, 0) for cell in far_candidates)

    def start_score(cell):
        distance_score = distances[cell] / max_distance if max_distance > 0 else 0.0
        clearance_score = (
            obstacle_distance.get(cell, 0) / max_clearance if max_clearance > 0 else 0.0
        )
        return clearance_score + 0.25 * distance_score

    start_cell = max(far_candidates, key=start_score)
    start_x, start_y = cell_to_world(start_cell)
    return f"{start_x:.2f}", f"{start_y:.2f}", "0.0"


def uniform_free_space_variance(map_yaml):
    metadata = _read_simple_yaml(map_yaml)
    image = Path(metadata["image"])
    if not image.is_absolute():
        image = map_yaml.parent / image

    width, height, max_value, data = read_p2_pgm(image)
    resolution = float(metadata.get("resolution", "0.1"))
    origin = _parse_origin(metadata.get("origin", "[0, 0, 0]"))
    origin_x = origin[0] if len(origin) > 0 else 0.0
    origin_y = origin[1] if len(origin) > 1 else 0.0
    free_value = max_value

    free_coords = []
    for row in range(height):
        for col in range(width):
            if data[row * width + col] == free_value:
                free_coords.append((
                    origin_x + (col + 0.5) * resolution,
                    origin_y + (height - row - 0.5) * resolution,
                ))

    if not free_coords:
        raise RuntimeError(f"No free cells found when calculating dynamic threshold for '{map_yaml}'.")

    mean_x = sum(x for x, _ in free_coords) / len(free_coords)
    mean_y = sum(y for _, y in free_coords) / len(free_coords)
    variance = sum(
        (x - mean_x) ** 2 + (y - mean_y) ** 2
        for x, y in free_coords
    ) / len(free_coords)

    return {
        "free_cells": len(free_coords),
        "free_area": len(free_coords) * resolution * resolution,
        "uniform_variance": variance,
        "uniform_std": math.sqrt(variance),
    }


def suggest_dynamic_convergence_threshold(map_yaml, target_error, alpha, max_error):
    stats = uniform_free_space_variance(map_yaml)
    target_threshold = target_error * target_error
    map_threshold = alpha * stats["uniform_variance"]
    max_threshold = max_error * max_error
    suggested_threshold = min(max(target_threshold, map_threshold), max_threshold)
    stats.update({
        "target_error": target_error,
        "alpha": alpha,
        "max_error": max_error,
        "target_threshold": target_threshold,
        "map_threshold": map_threshold,
        "max_threshold": max_threshold,
        "suggested_threshold": suggested_threshold,
        "suggested_uncertainty_radius": math.sqrt(suggested_threshold),
    })
    return stats


def resample_binary_map(width, height, data, source_resolution, target_resolution, free_value, blocked_value):
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
            # making already narrow indoor passages vanish completely.
            resampled.append(free_value if free_cells >= total_cells / 2 else blocked_value)

    return target_width, target_height, resampled


def make_navigation_safe_map(
    source_yaml,
    target_yaml,
    target_resolution=None,
    close_diagonal_gaps=True,
):
    metadata = _read_simple_yaml(source_yaml)
    source_image = Path(metadata["image"])
    if not source_image.is_absolute():
        source_image = source_yaml.parent / source_image

    width, height, max_value, data = read_p2_pgm(source_image)
    source_resolution = float(metadata.get("resolution", "0.1"))
    output_resolution = target_resolution or source_resolution
    free_value = max_value
    blocked_value = 0
    safe_data = data[:]
    blocked_cells = set()

    if close_diagonal_gaps:
        # Some VGR maps contain diagonal one-pixel gaps that look passable to a grid planner,
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

    output_width, output_height, output_data = resample_binary_map(
        width,
        height,
        safe_data,
        source_resolution,
        output_resolution,
        free_value,
        blocked_value,
    )

    target_image = target_yaml.with_name("navigation_occupancy.pgm")
    write_p2_pgm(target_image, output_width, output_height, max_value, output_data)
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
