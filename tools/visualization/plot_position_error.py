#!/usr/bin/env python3
"""Visualize GICI APE against an IE-style ground truth file."""

import argparse
import csv
import datetime as dt
import math
from pathlib import Path
from statistics import mean


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
GPS_EPOCH = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc)


def nmea_lat_lon(value, hemisphere):
    if not value:
        raise ValueError("empty NMEA coordinate")
    raw = float(value)
    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def parse_rmc_datetime(fields):
    time_text = fields[1]
    date_text = fields[9]
    if not time_text or not date_text:
        return None

    hour = int(time_text[0:2])
    minute = int(time_text[2:4])
    second_float = float(time_text[4:])
    second = int(second_float)
    microsecond = int(round((second_float - second) * 1_000_000))
    if microsecond == 1_000_000:
        second += 1
        microsecond = 0

    day = int(date_text[0:2])
    month = int(date_text[2:4])
    year = 2000 + int(date_text[4:6])
    return dt.datetime(
        year, month, day, hour, minute, second, microsecond, tzinfo=dt.timezone.utc
    )


def utc_seconds_of_week(timestamp):
    delta = timestamp - GPS_EPOCH
    return delta.total_seconds() % 604800.0


def parse_gici_nmea(path):
    samples = []
    current_time = None
    current_tow = None

    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line.startswith("$"):
                continue
            payload = line[1:].split("*", 1)[0]
            fields = payload.split(",")
            sentence = fields[0][-3:]

            if sentence == "RMC":
                current_time = parse_rmc_datetime(fields)
                current_tow = (
                    utc_seconds_of_week(current_time) if current_time is not None else None
                )
                continue

            if sentence == "ESA" and current_tow is not None and samples:
                if len(fields) >= 8 and fields[5] and fields[6] and fields[7]:
                    samples[-1]["solution_roll"] = float(fields[5])
                    samples[-1]["solution_pitch"] = float(fields[6])
                    samples[-1]["solution_yaw"] = float(fields[7])
                continue

            if sentence != "GGA" or current_tow is None:
                continue
            if len(fields) < 12 or not fields[2] or not fields[4]:
                continue

            lat = nmea_lat_lon(fields[2], fields[3])
            lon = nmea_lat_lon(fields[4], fields[5])
            fix_quality = int(fields[6]) if fields[6] else 0
            satellites = int(fields[7]) if fields[7] else 0
            altitude = float(fields[9]) if fields[9] else 0.0
            geoid_separation = float(fields[11]) if fields[11] else 0.0
            ellipsoid_height = altitude + geoid_separation

            samples.append(
                {
                    "utc_tow": current_tow,
                    "lat": lat,
                    "lon": lon,
                    "height": ellipsoid_height,
                    "fix_quality": fix_quality,
                    "satellites": satellites,
                }
            )

    return samples


def parse_truth(path):
    samples = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 6:
                continue
            try:
                int(parts[0])
                utc_tow = float(parts[2])
                lon = float(parts[3])
                lat = float(parts[4])
                height = float(parts[5])
                heading = float(parts[6]) if len(parts) > 6 else 0.0
                pitch = float(parts[7]) if len(parts) > 7 else 0.0
                roll = float(parts[8]) if len(parts) > 8 else 0.0
            except ValueError:
                continue
            samples.append(
                {
                    "utc_tow": utc_tow,
                    "lon": lon,
                    "lat": lat,
                    "height": height,
                    "heading": heading,
                    "pitch": pitch,
                    "roll": roll,
                }
            )

    samples.sort(key=lambda item: item["utc_tow"])
    return samples


def wrap_angle_deg(angle):
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def interpolate_angle_deg(left, right, ratio):
    return wrap_angle_deg(left + wrap_angle_deg(right - left) * ratio)


def interpolate_truth(truth, utc_tow, start_index):
    index = start_index
    while index + 1 < len(truth) and truth[index + 1]["utc_tow"] < utc_tow:
        index += 1

    if index + 1 >= len(truth):
        return None, index

    left = truth[index]
    right = truth[index + 1]
    if not (left["utc_tow"] <= utc_tow <= right["utc_tow"]):
        return None, index

    span = right["utc_tow"] - left["utc_tow"]
    ratio = 0.0 if span == 0.0 else (utc_tow - left["utc_tow"]) / span
    interpolated = {
        "utc_tow": utc_tow,
        "lon": left["lon"] + (right["lon"] - left["lon"]) * ratio,
        "lat": left["lat"] + (right["lat"] - left["lat"]) * ratio,
        "height": left["height"] + (right["height"] - left["height"]) * ratio,
    }
    if "heading" in left and "heading" in right:
        interpolated["heading"] = interpolate_angle_deg(left["heading"], right["heading"], ratio)
        interpolated["pitch"] = interpolate_angle_deg(left["pitch"], right["pitch"], ratio)
        interpolated["roll"] = interpolate_angle_deg(left["roll"], right["roll"], ratio)
    return interpolated, index


def radii_of_curvature(lat_rad):
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    n_radius = WGS84_A / denom
    m_radius = WGS84_A * (1.0 - WGS84_E2) / (denom**3)
    return m_radius, n_radius


def enu_error(solution, truth):
    lat_ref = math.radians(truth["lat"])
    lon_ref = math.radians(truth["lon"])
    lat = math.radians(solution["lat"])
    lon = math.radians(solution["lon"])
    m_radius, n_radius = radii_of_curvature(lat_ref)

    east = (lon - lon_ref) * (n_radius + truth["height"]) * math.cos(lat_ref)
    north = (lat - lat_ref) * (m_radius + truth["height"])
    up = solution["height"] - truth["height"]
    horizontal = math.hypot(east, north)
    spatial = math.sqrt(east * east + north * north + up * up)
    return east, north, up, horizontal, spatial


def enu_offset(sample, origin):
    lat_ref = math.radians(origin["lat"])
    lon_ref = math.radians(origin["lon"])
    lat = math.radians(sample["lat"])
    lon = math.radians(sample["lon"])
    m_radius, n_radius = radii_of_curvature(lat_ref)

    east = (lon - lon_ref) * (n_radius + origin["height"]) * math.cos(lat_ref)
    north = (lat - lat_ref) * (m_radius + origin["height"])
    up = sample["height"] - origin["height"]
    return east, north, up


def add_enu_offset(sample, east, north, up):
    lat_ref = math.radians(sample["lat"])
    lon_ref = math.radians(sample["lon"])
    m_radius, n_radius = radii_of_curvature(lat_ref)

    lat = lat_ref + north / (m_radius + sample["height"])
    lon = lon_ref + east / ((n_radius + sample["height"]) * math.cos(lat_ref))
    adjusted = dict(sample)
    adjusted["lat"] = math.degrees(lat)
    adjusted["lon"] = math.degrees(lon)
    adjusted["height"] = sample["height"] + up
    return adjusted


def identity_matrix():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def mat_mul(left, right):
    return [
        [
            sum(left[row][index] * right[index][col] for index in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def mat_transpose(matrix):
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def mat_vec_mul(matrix, vector):
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def rotation_x(angle_rad):
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return [
        [1.0, 0.0, 0.0],
        [0.0, cos_a, -sin_a],
        [0.0, sin_a, cos_a],
    ]


def rotation_y(angle_rad):
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return [
        [cos_a, 0.0, sin_a],
        [0.0, 1.0, 0.0],
        [-sin_a, 0.0, cos_a],
    ]


def rotation_z(angle_rad):
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return [
        [cos_a, -sin_a, 0.0],
        [sin_a, cos_a, 0.0],
        [0.0, 0.0, 1.0],
    ]


def euler_rpy_matrix(roll_deg, pitch_deg, yaw_deg):
    return mat_mul(
        mat_mul(rotation_z(math.radians(yaw_deg)), rotation_y(math.radians(pitch_deg))),
        rotation_x(math.radians(roll_deg)),
    )


def rotation_error_deg(solution_rotation, truth_rotation):
    delta = mat_mul(mat_transpose(solution_rotation), truth_rotation)
    trace = delta[0][0] + delta[1][1] + delta[2][2]
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_angle))


def ground_truth_rotation_enu(truth):
    heading = math.radians(truth["heading"])
    left = [-math.cos(heading), math.sin(heading), 0.0]
    forward = [math.sin(heading), math.cos(heading), 0.0]
    down = [0.0, 0.0, -1.0]
    heading_matrix = [
        [left[0], forward[0], down[0]],
        [left[1], forward[1], down[1]],
        [left[2], forward[2], down[2]],
    ]
    return mat_mul(
        mat_mul(heading_matrix, rotation_x(math.radians(-truth["pitch"]))),
        rotation_y(math.radians(wrap_angle_deg(truth["roll"] + 180.0))),
    )


def ground_truth_body_rotation_enu(truth, t_b_gt):
    return mat_mul(ground_truth_rotation_enu(truth), mat_transpose(t_b_gt["rotation"]))


def body_offset_to_enu(t_b_gt, truth):
    return mat_vec_mul(ground_truth_body_rotation_enu(truth, t_b_gt), t_b_gt["translation"])


def legacy_body_offset_to_enu(t_body_gt, heading_deg):
    heading = math.radians(heading_deg)
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    right, forward, up = t_body_gt
    east = cos_h * right + sin_h * forward
    north = -sin_h * right + cos_h * forward
    return east, north, up


def ground_truth_to_body_reference(truth, t_b_gt):
    for key in ("heading", "pitch", "roll"):
        if key not in truth:
            raise ValueError("Ground truth lever-arm correction requires heading/pitch/roll in truth samples")
    gt_east, gt_north, gt_up = body_offset_to_enu(t_b_gt, truth)
    return add_enu_offset(truth, -gt_east, -gt_north, -gt_up)


def extract_yaml_matrix_data(path, key):
    lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    for index, line in enumerate(lines):
        if line.strip() != f"{key}:":
            continue
        data_text = ""
        for candidate in lines[index + 1 :]:
            stripped = candidate.strip()
            if data_text and stripped and not stripped.startswith(("data:", "-", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9")):
                break
            if stripped.startswith("data:"):
                data_text = stripped.split("data:", 1)[1]
            elif data_text:
                data_text += " " + stripped
            if data_text and "]" in data_text:
                payload = data_text.split("[", 1)[1].split("]", 1)[0]
                return [float(value.strip()) for value in payload.split(",") if value.strip()]
    raise ValueError(f"Unable to find matrix data for {key} in {path}")


def load_body_from_ground_truth_transform(path):
    data = extract_yaml_matrix_data(path, "T_B_GT")
    if len(data) != 16:
        raise ValueError(f"T_B_GT must contain 16 values, got {len(data)}")
    return {
        "rotation": [
            [data[0], data[1], data[2]],
            [data[4], data[5], data[6]],
            [data[8], data[9], data[10]],
        ],
        "translation": [data[3], data[7], data[11]],
    }


def attitude_error(solution, truth, t_b_gt=None):
    if not all(key in solution for key in ("solution_roll", "solution_pitch", "solution_yaw")):
        return None
    if t_b_gt is None:
        return None
    solution_rotation = euler_rpy_matrix(
        solution["solution_roll"],
        solution["solution_pitch"],
        solution["solution_yaw"],
    )
    truth_rotation = ground_truth_body_rotation_enu(truth, t_b_gt)
    return rotation_error_deg(solution_rotation, truth_rotation)


def align_errors(solution_samples, truth_samples, t_b_gt=None):
    rows = []
    truth_index = 0
    origin = None
    for sample in solution_samples:
        truth, truth_index = interpolate_truth(truth_samples, sample["utc_tow"], truth_index)
        if truth is None:
            continue
        raw_truth = truth
        if t_b_gt is not None:
            truth = ground_truth_to_body_reference(truth, t_b_gt)
        if origin is None:
            origin = truth
        east, north, up, horizontal, spatial = enu_error(sample, truth)
        solution_east, solution_north, solution_up = enu_offset(sample, origin)
        truth_east, truth_north, truth_up = enu_offset(truth, origin)
        rows.append(
            {
                "utc_tow": sample["utc_tow"],
                "elapsed": sample["utc_tow"] - solution_samples[0]["utc_tow"],
                "solution_lon": sample["lon"],
                "solution_lat": sample["lat"],
                "solution_height": sample["height"],
                "truth_lon": truth["lon"],
                "truth_lat": truth["lat"],
                "truth_height": truth["height"],
                "solution_east_m": solution_east,
                "solution_north_m": solution_north,
                "solution_up_m": solution_up,
                "truth_east_m": truth_east,
                "truth_north_m": truth_north,
                "truth_up_m": truth_up,
                "east_error_m": east,
                "north_error_m": north,
                "up_error_m": up,
                "horizontal_error_m": horizontal,
                "spatial_error_m": spatial,
                "ape_translation_m": spatial,
                "ape_horizontal_m": horizontal,
                "attitude_error_deg": attitude_error(sample, raw_truth, t_b_gt),
                "fix_quality": sample["fix_quality"],
                "satellites": sample["satellites"],
            }
        )
    return rows


def rms(values):
    return math.sqrt(mean([value * value for value in values]))


def summarize(rows):
    metrics = {}
    for key in [
        "ape_translation_m",
        "ape_horizontal_m",
        "attitude_error_deg",
        "east_error_m",
        "north_error_m",
        "up_error_m",
    ]:
        values = [row[key] for row in rows if row.get(key) is not None]
        if not values:
            continue
        metrics[key] = {
            "mean": mean(values),
            "rms": rms(values),
            "max_abs": max(abs(value) for value in values),
        }
    return metrics


def write_csv(path, rows):
    fieldnames = [
        "utc_tow",
        "elapsed",
        "solution_lon",
        "solution_lat",
        "solution_height",
        "truth_lon",
        "truth_lat",
        "truth_height",
        "solution_east_m",
        "solution_north_m",
        "solution_up_m",
        "truth_east_m",
        "truth_north_m",
        "truth_up_m",
        "ape_translation_m",
        "ape_horizontal_m",
        "attitude_error_deg",
        "east_error_m",
        "north_error_m",
        "up_error_m",
        "horizontal_error_m",
        "spatial_error_m",
        "fix_quality",
        "satellites",
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def parse_evaluation_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row = {}
            for key, value in raw.items():
                if key == "solution_status":
                    row[key] = value
                elif value == "":
                    row[key] = 0.0
                else:
                    row[key] = float(value)

            row["utc_tow"] = row["timestamp_gpst"]
            row["elapsed"] = row["elapsed_s"]
            row["solution_lon"] = row["solution_lon_deg"]
            row["solution_lat"] = row["solution_lat_deg"]
            row["solution_height"] = row["solution_height_m"]
            row["truth_lon"] = row["truth_lon_deg"]
            row["truth_lat"] = row["truth_lat_deg"]
            row["truth_height"] = row["truth_height_m"]
            row["ape_translation_m"] = row["translation_error_m"]
            row["ape_horizontal_m"] = row["horizontal_error_m"]
            row["spatial_error_m"] = row["translation_error_m"]
            row["solution_roll"] = row.get("solution_roll_deg", 0.0)
            row["solution_pitch"] = row.get("solution_pitch_deg", 0.0)
            row["solution_yaw"] = row.get("solution_yaw_deg", 0.0)
            row["fix_quality"] = row.get("solution_status", "")
            row["satellites"] = row.get("num_satellites", 0.0)
            rows.append(row)
    return rows


def apply_ground_truth_transform_to_evaluation_rows(rows, t_b_gt):
    if not rows:
        return rows
    origin = None
    adjusted_rows = []
    for row in rows:
        raw_truth = {
            "lon": row["truth_lon"],
            "lat": row["truth_lat"],
            "height": row["truth_height"],
            "heading": row["truth_heading_deg"],
            "pitch": row["truth_pitch_deg"],
            "roll": row["truth_roll_deg"],
        }
        truth = ground_truth_to_body_reference(raw_truth, t_b_gt)
        solution = {
            "lon": row["solution_lon"],
            "lat": row["solution_lat"],
            "height": row["solution_height"],
            "solution_roll": row["solution_roll_deg"],
            "solution_pitch": row["solution_pitch_deg"],
            "solution_yaw": row["solution_yaw_deg"],
        }
        if origin is None:
            origin = truth
        east, north, up, horizontal, spatial = enu_error(solution, truth)
        solution_east, solution_north, solution_up = enu_offset(solution, origin)
        truth_east, truth_north, truth_up = enu_offset(truth, origin)

        adjusted = dict(row)
        adjusted["truth_lon"] = truth["lon"]
        adjusted["truth_lat"] = truth["lat"]
        adjusted["truth_height"] = truth["height"]
        adjusted["truth_lon_deg"] = truth["lon"]
        adjusted["truth_lat_deg"] = truth["lat"]
        adjusted["truth_height_m"] = truth["height"]
        adjusted["solution_east_m"] = solution_east
        adjusted["solution_north_m"] = solution_north
        adjusted["solution_up_m"] = solution_up
        adjusted["truth_east_m"] = truth_east
        adjusted["truth_north_m"] = truth_north
        adjusted["truth_up_m"] = truth_up
        adjusted["east_error_m"] = east
        adjusted["north_error_m"] = north
        adjusted["up_error_m"] = up
        adjusted["horizontal_error_m"] = horizontal
        adjusted["translation_error_m"] = spatial
        adjusted["spatial_error_m"] = spatial
        adjusted["ape_translation_m"] = spatial
        adjusted["ape_horizontal_m"] = horizontal
        adjusted["attitude_error_deg"] = attitude_error(solution, raw_truth, t_b_gt)
        adjusted_rows.append(adjusted)
    return adjusted_rows


def scale(value, src_min, src_max, dst_min, dst_max):
    if src_max == src_min:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) * (dst_max - dst_min) / (src_max - src_min)


def polyline(points, x_min, x_max, y_min, y_max, left, top, width, height):
    output = []
    for x_value, y_value in points:
        x = scale(x_value, x_min, x_max, left, left + width)
        y = scale(y_value, y_min, y_max, top + height, top)
        output.append(f"{x:.2f},{y:.2f}")
    return " ".join(output)


def write_ape_svg(path, rows, metrics):
    width = 1200
    panel_height = 230
    margin_left = 82
    margin_right = 36
    margin_top = 62
    panel_gap = 42
    plot_width = width - margin_left - margin_right
    height = margin_top + 3 * panel_height + 2 * panel_gap + 72
    x_min = rows[0]["elapsed"]
    x_max = rows[-1]["elapsed"]
    y_values = [
        row[key]
        for row in rows
        for key in ("ape_translation_m", "ape_horizontal_m", "up_error_m")
    ]
    y_abs = max(max(abs(value) for value in y_values), 0.05)
    y_limit = math.ceil(y_abs * 10.0) / 10.0

    series = [
        ("APE translation", "ape_translation_m", "#111111"),
        ("APE horizontal", "ape_horizontal_m", "#1f77b4"),
        ("Up error", "up_error_m", "#d62728"),
    ]

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#222}",
        ".axis{stroke:#333;stroke-width:1}",
        ".grid{stroke:#ddd;stroke-width:1}",
        ".zero{stroke:#777;stroke-width:1;stroke-dasharray:5 5}",
        ".label{font-size:14px}",
        ".title{font-size:24px;font-weight:700}",
        ".small{font-size:12px;fill:#555}",
        "</style>",
        '<rect width="100%" height="100%" fill="#fff"/>',
        f'<text class="title" x="{margin_left}" y="34">GICI APE vs Ground Truth</text>',
        f'<text class="small" x="{margin_left}" y="54">Samples: {len(rows)}, time span: {x_min:.1f}s to {x_max:.1f}s</text>',
    ]

    for panel_index, (name, key, color) in enumerate(series):
        top = margin_top + panel_index * (panel_height + panel_gap)
        bottom = top + panel_height
        lines.append(f'<line class="axis" x1="{margin_left}" y1="{bottom}" x2="{margin_left + plot_width}" y2="{bottom}"/>')
        lines.append(f'<line class="axis" x1="{margin_left}" y1="{top}" x2="{margin_left}" y2="{bottom}"/>')
        for tick in [-y_limit, -y_limit / 2.0, 0.0, y_limit / 2.0, y_limit]:
            y = scale(tick, -y_limit, y_limit, bottom, top)
            class_name = "zero" if abs(tick) < 1e-12 else "grid"
            lines.append(f'<line class="{class_name}" x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + plot_width}" y2="{y:.2f}"/>')
            lines.append(f'<text class="small" x="{margin_left - 10}" y="{y + 4:.2f}" text-anchor="end">{tick:.2f}</text>')
        for i in range(6):
            t = x_min + (x_max - x_min) * i / 5.0
            x = scale(t, x_min, x_max, margin_left, margin_left + plot_width)
            lines.append(f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{bottom}"/>')
            if panel_index == 2:
                lines.append(f'<text class="small" x="{x:.2f}" y="{bottom + 22}" text-anchor="middle">{t:.0f}</text>')

        points = [(row["elapsed"], row[key]) for row in rows]
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="1.8" points="{polyline(points, x_min, x_max, -y_limit, y_limit, margin_left, top, plot_width, panel_height)}"/>'
        )
        stat = metrics[key]
        lines.append(f'<text class="label" x="20" y="{top + 26}" font-weight="700">{name}</text>')
        lines.append(f'<text class="small" x="{margin_left + 8}" y="{top + 18}">mean={stat["mean"]:.4f} m, rms={stat["rms"]:.4f} m, max_abs={stat["max_abs"]:.4f} m</text>')

    lines.append(f'<text class="label" x="{width / 2:.0f}" y="{height - 22}" text-anchor="middle">Elapsed time (s)</text>')
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_component_svg(path, rows, metrics):
    width = 1200
    panel_height = 230
    margin_left = 82
    margin_right = 36
    margin_top = 62
    panel_gap = 42
    plot_width = width - margin_left - margin_right
    height = margin_top + 3 * panel_height + 2 * panel_gap + 72
    x_min = rows[0]["elapsed"]
    x_max = rows[-1]["elapsed"]
    y_values = [row[key] for row in rows for key in ("east_error_m", "north_error_m", "up_error_m")]
    y_abs = max(max(abs(value) for value in y_values), 0.05)
    y_limit = math.ceil(y_abs * 10.0) / 10.0

    series = [
        ("East", "east_error_m", "#1f77b4"),
        ("North", "north_error_m", "#2ca02c"),
        ("Up", "up_error_m", "#d62728"),
    ]

    lines = svg_header(width, height)
    lines.extend(
        [
            f'<text class="title" x="{margin_left}" y="34">ENU Error Components</text>',
            f'<text class="small" x="{margin_left}" y="54">Positive axes: east, north, up. Units: meters.</text>',
        ]
    )

    append_timeseries_panels(
        lines,
        rows,
        metrics,
        series,
        x_min,
        x_max,
        -y_limit,
        y_limit,
        margin_left,
        margin_top,
        plot_width,
        panel_height,
        panel_gap,
    )
    lines.append(f'<text class="label" x="{width / 2:.0f}" y="{height - 22}" text-anchor="middle">Elapsed time (s)</text>')
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def svg_header(width, height):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#222}",
        ".axis{stroke:#333;stroke-width:1}",
        ".grid{stroke:#ddd;stroke-width:1}",
        ".zero{stroke:#777;stroke-width:1;stroke-dasharray:5 5}",
        ".label{font-size:14px}",
        ".title{font-size:24px;font-weight:700}",
        ".small{font-size:12px;fill:#555}",
        "</style>",
        '<rect width="100%" height="100%" fill="#fff"/>',
    ]


def append_timeseries_panels(
    lines,
    rows,
    metrics,
    series,
    x_min,
    x_max,
    y_min,
    y_max,
    margin_left,
    margin_top,
    plot_width,
    panel_height,
    panel_gap,
):
    for panel_index, (name, key, color) in enumerate(series):
        top = margin_top + panel_index * (panel_height + panel_gap)
        bottom = top + panel_height
        lines.append(f'<line class="axis" x1="{margin_left}" y1="{bottom}" x2="{margin_left + plot_width}" y2="{bottom}"/>')
        lines.append(f'<line class="axis" x1="{margin_left}" y1="{top}" x2="{margin_left}" y2="{bottom}"/>')
        for tick in [y_min, (y_min + y_max) / 4.0, (y_min + y_max) / 2.0, (y_min + y_max) * 3.0 / 4.0, y_max]:
            y = scale(tick, y_min, y_max, bottom, top)
            class_name = "zero" if abs(tick) < 1e-12 else "grid"
            lines.append(f'<line class="{class_name}" x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + plot_width}" y2="{y:.2f}"/>')
            lines.append(f'<text class="small" x="{margin_left - 10}" y="{y + 4:.2f}" text-anchor="end">{tick:.2f}</text>')
        for i in range(6):
            t = x_min + (x_max - x_min) * i / 5.0
            x = scale(t, x_min, x_max, margin_left, margin_left + plot_width)
            lines.append(f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{bottom}"/>')
            if panel_index == len(series) - 1:
                lines.append(f'<text class="small" x="{x:.2f}" y="{bottom + 22}" text-anchor="middle">{t:.0f}</text>')

        points = [(row["elapsed"], row[key]) for row in rows]
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="1.8" points="{polyline(points, x_min, x_max, y_min, y_max, margin_left, top, plot_width, panel_height)}"/>'
        )
        stat = metrics[key]
        lines.append(f'<text class="label" x="20" y="{top + 26}" font-weight="700">{name}</text>')
        lines.append(f'<text class="small" x="{margin_left + 8}" y="{top + 18}">mean={stat["mean"]:.4f} m, rms={stat["rms"]:.4f} m, max_abs={stat["max_abs"]:.4f} m</text>')


def write_trajectory_svg(path, rows):
    width = 1000
    height = 820
    left = 86
    top = 70
    plot_width = width - left - 50
    plot_height = height - top - 90
    all_e = [row["solution_east_m"] for row in rows] + [row["truth_east_m"] for row in rows]
    all_n = [row["solution_north_m"] for row in rows] + [row["truth_north_m"] for row in rows]
    e_min, e_max = min(all_e), max(all_e)
    n_min, n_max = min(all_n), max(all_n)
    pad_e = max((e_max - e_min) * 0.06, 1.0)
    pad_n = max((n_max - n_min) * 0.06, 1.0)
    e_min -= pad_e
    e_max += pad_e
    n_min -= pad_n
    n_max += pad_n

    lines = svg_header(width, height)
    lines.extend(
        [
            f'<text class="title" x="{left}" y="34">Trajectory Comparison</text>',
            f'<text class="small" x="{left}" y="54">Local EN plane, origin at first aligned ground truth sample.</text>',
            f'<line class="axis" x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}"/>',
            f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}"/>',
        ]
    )

    for i in range(6):
        e_tick = e_min + (e_max - e_min) * i / 5.0
        x = scale(e_tick, e_min, e_max, left, left + plot_width)
        lines.append(f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}"/>')
        lines.append(f'<text class="small" x="{x:.2f}" y="{top + plot_height + 24}" text-anchor="middle">{e_tick:.0f}</text>')
        n_tick = n_min + (n_max - n_min) * i / 5.0
        y = scale(n_tick, n_min, n_max, top + plot_height, top)
        lines.append(f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}"/>')
        lines.append(f'<text class="small" x="{left - 10}" y="{y + 4:.2f}" text-anchor="end">{n_tick:.0f}</text>')

    truth_points = [
        (row["truth_east_m"], row["truth_north_m"])
        for row in rows
    ]
    solution_points = [
        (row["solution_east_m"], row["solution_north_m"])
        for row in rows
    ]
    lines.append(
        f'<polyline fill="none" stroke="#222" stroke-width="2.2" points="{polyline(truth_points, e_min, e_max, n_min, n_max, left, top, plot_width, plot_height)}"/>'
    )
    lines.append(
        f'<polyline fill="none" stroke="#d62728" stroke-width="1.8" points="{polyline(solution_points, e_min, e_max, n_min, n_max, left, top, plot_width, plot_height)}"/>'
    )
    lines.extend(
        [
            f'<text class="label" x="{width / 2:.0f}" y="{height - 24}" text-anchor="middle">East (m)</text>',
            f'<text class="label" x="24" y="{height / 2:.0f}" transform="rotate(-90 24 {height / 2:.0f})" text-anchor="middle">North (m)</text>',
            f'<line x1="{left + 20}" y1="{top + 22}" x2="{left + 70}" y2="{top + 22}" stroke="#222" stroke-width="2.2"/>',
            f'<text class="small" x="{left + 80}" y="{top + 26}">Ground truth</text>',
            f'<line x1="{left + 20}" y1="{top + 44}" x2="{left + 70}" y2="{top + 44}" stroke="#d62728" stroke-width="1.8"/>',
            f'<text class="small" x="{left + 80}" y="{top + 48}">GICI solution</text>',
        ]
    )
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_summary(path, rows, metrics):
    lines = [
        "GICI APE summary",
        f"samples: {len(rows)}",
        f"utc_tow_start: {rows[0]['utc_tow']:.3f}",
        f"utc_tow_end: {rows[-1]['utc_tow']:.3f}",
        f"duration_s: {rows[-1]['elapsed']:.3f}",
        "",
        "Metric notes:",
        "ape_translation_m: preferred APE metric, 3D translation norm sqrt(E^2 + N^2 + U^2).",
        "ape_horizontal_m: 2D trajectory APE on the local East/North plane.",
        "attitude_error_deg: SO(3) rotation angle after converting truth with full HPR attitude and T_B_GT.",
        "east_error_m/north_error_m/up_error_m: signed ENU components. They explain direction and source of APE.",
        "mean: arithmetic average. For signed ENU components it can hide opposite-direction errors.",
        "rms: root mean square, useful as the main accuracy statistic.",
        "max_abs: largest absolute error observed in the aligned samples.",
        "",
    ]
    for key, values in metrics.items():
        unit = "deg" if key == "attitude_error_deg" else "m"
        lines.append(
            f"{key}: mean={values['mean']:.6f} {unit}, rms={values['rms']:.6f} {unit}, max_abs={values['max_abs']:.6f} {unit}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation-csv", default=None)
    parser.add_argument("--solution", default="output/rtk_tc_solution.txt")
    parser.add_argument("--truth", default="/home/wbz/桌面/GICI/data/1.1/ground_truth.txt")
    parser.add_argument("--extrinsics-yaml", default=None)
    parser.add_argument("--output-dir", default="tools/visualization/output")
    return parser.parse_args()


def main():
    args = parse_args()
    solution_path = Path(args.solution)
    truth_path = Path(args.truth)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.evaluation_csv:
        rows = parse_evaluation_csv(Path(args.evaluation_csv))
        if args.extrinsics_yaml:
            rows = apply_ground_truth_transform_to_evaluation_rows(
                rows,
                load_body_from_ground_truth_transform(Path(args.extrinsics_yaml)),
            )
    else:
        solution_samples = parse_gici_nmea(solution_path)
        truth_samples = parse_truth(truth_path)
        if not solution_samples:
            raise SystemExit(f"No GGA/RMC solution samples found in {solution_path}")
        if len(truth_samples) < 2:
            raise SystemExit(f"Not enough truth samples found in {truth_path}")
        t_b_gt = (
            load_body_from_ground_truth_transform(Path(args.extrinsics_yaml))
            if args.extrinsics_yaml
            else None
        )
        rows = align_errors(solution_samples, truth_samples, t_b_gt=t_b_gt)

    if not rows:
        raise SystemExit("No overlapping timestamps between solution and truth")

    metrics = summarize(rows)
    write_csv(output_dir / "ape.csv", rows)
    write_ape_svg(output_dir / "ape_timeseries.svg", rows, metrics)
    write_component_svg(output_dir / "enu_components.svg", rows, metrics)
    write_trajectory_svg(output_dir / "trajectory_comparison.svg", rows)
    write_summary(output_dir / "summary.txt", rows, metrics)

    print(f"Aligned samples: {len(rows)}")
    print(f"Time span: {rows[0]['utc_tow']:.3f} -> {rows[-1]['utc_tow']:.3f} seconds-of-week")
    for key in ("ape_translation_m", "ape_horizontal_m", "attitude_error_deg", "east_error_m", "north_error_m", "up_error_m"):
        if key not in metrics:
            continue
        values = metrics[key]
        unit = "deg" if key == "attitude_error_deg" else "m"
        print(
            f"{key}: mean={values['mean']:.4f} {unit}, rms={values['rms']:.4f} {unit}, max_abs={values['max_abs']:.4f} {unit}"
        )
    print(f"Wrote {output_dir / 'ape_timeseries.svg'}")
    print(f"Wrote {output_dir / 'trajectory_comparison.svg'}")


if __name__ == "__main__":
    main()
