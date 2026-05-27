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
            except ValueError:
                continue
            samples.append({"utc_tow": utc_tow, "lon": lon, "lat": lat, "height": height})

    samples.sort(key=lambda item: item["utc_tow"])
    return samples


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


def align_errors(solution_samples, truth_samples):
    rows = []
    truth_index = 0
    origin = None
    for sample in solution_samples:
        truth, truth_index = interpolate_truth(truth_samples, sample["utc_tow"], truth_index)
        if truth is None:
            continue
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
        "east_error_m",
        "north_error_m",
        "up_error_m",
    ]:
        values = [row[key] for row in rows]
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
            row["fix_quality"] = row.get("solution_status", "")
            row["satellites"] = row.get("num_satellites", 0.0)
            rows.append(row)
    return rows


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
        "east_error_m/north_error_m/up_error_m: signed ENU components. They explain direction and source of APE.",
        "mean: arithmetic average. For signed ENU components it can hide opposite-direction errors.",
        "rms: root mean square, useful as the main accuracy statistic.",
        "max_abs: largest absolute error observed in the aligned samples.",
        "",
    ]
    for key, values in metrics.items():
        lines.append(
            f"{key}: mean={values['mean']:.6f} m, rms={values['rms']:.6f} m, max_abs={values['max_abs']:.6f} m"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation-csv", default=None)
    parser.add_argument("--solution", default="output/rtk_tc_solution.txt")
    parser.add_argument("--truth", default="/home/wbz/桌面/GICI/data/1.1/ground_truth.txt")
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
    else:
        solution_samples = parse_gici_nmea(solution_path)
        truth_samples = parse_truth(truth_path)
        if not solution_samples:
            raise SystemExit(f"No GGA/RMC solution samples found in {solution_path}")
        if len(truth_samples) < 2:
            raise SystemExit(f"Not enough truth samples found in {truth_path}")
        rows = align_errors(solution_samples, truth_samples)

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
    for key in ("ape_translation_m", "ape_horizontal_m", "east_error_m", "north_error_m", "up_error_m"):
        values = metrics[key]
        print(
            f"{key}: mean={values['mean']:.4f} m, rms={values['rms']:.4f} m, max_abs={values['max_abs']:.4f} m"
        )
    print(f"Wrote {output_dir / 'ape_timeseries.svg'}")
    print(f"Wrote {output_dir / 'trajectory_comparison.svg'}")


if __name__ == "__main__":
    main()
