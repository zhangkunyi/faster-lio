#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze a PCD using the ground plane and wall directions. "
            "Default mode outputs roll/pitch from the ground and yaw from walls."
        )
    )
    parser.add_argument(
        "--pcd",
        nargs="?",
        default=Path(__file__).resolve().parents[1] / "PCD" / "map_for_localization.pcd",
        type=Path,
        help="Path to the PCD file. Defaults to faster-lio/PCD/map_for_localization.pcd",
    )
    parser.add_argument(
        "--mode",
        choices=("all", "ground", "yaw"),
        default="all",
        help=(
            "`all` outputs roll/pitch from the ground plus yaw from walls; "
            "`ground` only estimates ground tilt; `yaw` only estimates heading around z from wall directions."
        ),
    )
    parser.add_argument(
        "--ground-percentile",
        type=float,
        default=10.0,
        help="Ground mode: lowest z percentile used as the start of the ground search range. Default: 10",
    )
    parser.add_argument(
        "--ground-max-percentile",
        type=float,
        default=50.0,
        help="Ground mode: highest z percentile used during ground search. Default: 50",
    )
    parser.add_argument(
        "--ground-scan-steps",
        type=int,
        default=9,
        help="Ground mode: number of z-percentile cutoffs scanned between min and max percentile. Default: 9",
    )
    parser.add_argument(
        "--max-ground-tilt-deg",
        type=float,
        default=15.0,
        help="Ground mode: discard candidate planes whose tilt exceeds this value. Default: 15",
    )
    parser.add_argument(
        "--sample-limit",
        type=int,
        default=30000,
        help="Maximum sampled points used during fitting. Default: 30000",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=400,
        help="RANSAC iterations for ground mode. Default: 400",
    )
    parser.add_argument(
        "--inlier-threshold",
        type=float,
        default=0.08,
        help="Ground mode: plane inlier distance threshold in meters. Default: 0.08",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for reproducible sampling. Default: 0",
    )
    parser.add_argument(
        "--xy-grid",
        type=float,
        default=0.2,
        help="Yaw mode: XY grid size in meters for vertical-structure extraction. Default: 0.2",
    )
    parser.add_argument(
        "--min-z-span",
        type=float,
        default=1.5,
        help="Yaw mode: minimum z span per XY cell to be treated as wall-like. Default: 1.5",
    )
    parser.add_argument(
        "--min-cell-points",
        type=int,
        default=3,
        help="Yaw mode: minimum points in an XY cell. Default: 3",
    )
    parser.add_argument(
        "--neighbor-radius",
        type=float,
        default=0.8,
        help="Yaw mode: neighbor radius in meters for local direction fitting. Default: 0.8",
    )
    parser.add_argument(
        "--min-neighbors",
        type=int,
        default=5,
        help="Yaw mode: minimum nearby wall cells for local PCA. Default: 5",
    )
    parser.add_argument(
        "--linearity-ratio",
        type=float,
        default=4.0,
        help="Yaw mode: require major/minor eigenvalue ratio above this value. Default: 4.0",
    )
    parser.add_argument(
        "--hist-bin-deg",
        type=float,
        default=1.0,
        help="Yaw mode: histogram bin width in degrees over [-90, 90). Default: 1.0",
    )
    return parser.parse_args()


def read_pcd_xyz(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Invalid PCD: missing DATA header")
            decoded = line.decode("ascii", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                data_kind = decoded.split()[1].strip().lower()
                data_offset = f.tell()
                break

    meta = {}
    for line in header_lines:
        parts = line.split()
        if parts:
            meta[parts[0]] = parts[1:]

    fields = meta.get("FIELDS")
    sizes = list(map(int, meta.get("SIZE", [])))
    types = meta.get("TYPE", [])
    counts = list(map(int, meta.get("COUNT", ["1"] * len(fields))))
    points = int(meta["POINTS"][0])

    if fields is None or not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must contain x/y/z fields")

    if data_kind == "ascii":
        data = np.loadtxt(
            path,
            skiprows=len(header_lines),
            usecols=[fields.index("x"), fields.index("y"), fields.index("z")],
        )
        xyz = np.atleast_2d(data)
        return xyz[np.isfinite(xyz).all(axis=1)]

    if data_kind != "binary":
        raise ValueError(f"Unsupported PCD DATA type: {data_kind}")

    def scalar_dtype(name: str, size: int, typ: str):
        if typ == "F" and size == 4:
            return "<f4"
        if typ == "F" and size == 8:
            return "<f8"
        if typ == "I" and size == 1:
            return "i1"
        if typ == "I" and size == 2:
            return "<i2"
        if typ == "I" and size == 4:
            return "<i4"
        if typ == "U" and size == 1:
            return "u1"
        if typ == "U" and size == 2:
            return "<u2"
        if typ == "U" and size == 4:
            return "<u4"
        raise ValueError(f"Unsupported field format: {name} {typ}{size}")

    xyz_names = []
    xyz_formats = []
    xyz_offsets = []
    offset = 0
    for name, size, typ, count in zip(fields, sizes, types, counts):
        if name in {"x", "y", "z"}:
            if count != 1:
                raise ValueError(f"PCD coordinate field {name} must have COUNT 1, got {count}")
            xyz_names.append(name)
            xyz_formats.append(scalar_dtype(name, size, typ))
            xyz_offsets.append(offset)
        offset += size * count

    raw = np.fromfile(
        path,
        dtype=np.dtype(
            {
                "names": xyz_names,
                "formats": xyz_formats,
                "offsets": xyz_offsets,
                "itemsize": offset,
            }
        ),
        offset=data_offset,
        count=points,
    )
    xyz = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float64, copy=False)
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_plane_svd(points: np.ndarray):
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    d = -np.dot(normal, centroid)
    return normal, d


def plane_distances(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    return np.abs(points @ normal + d)


def ransac_plane(points: np.ndarray, iterations: int, threshold: float, seed: int):
    rng = np.random.default_rng(seed)
    best_inlier_mask = None
    best_count = -1
    best_error = float("inf")

    for _ in range(iterations):
        sample_idx = rng.choice(len(points), size=3, replace=False)
        p0, p1, p2 = points[sample_idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        d = -np.dot(normal, p0)

        distances = plane_distances(points, normal, d)
        inlier_mask = distances < threshold
        count = int(inlier_mask.sum())
        if count < 3:
            continue

        error = float(distances[inlier_mask].mean())
        if count > best_count or (count == best_count and error < best_error):
            best_inlier_mask = inlier_mask
            best_count = count
            best_error = error

    if best_inlier_mask is None:
        raise RuntimeError("RANSAC failed to find a valid plane")

    inliers = points[best_inlier_mask]
    normal, d = fit_plane_svd(inliers)
    distances = plane_distances(points, normal, d)
    refined_inlier_mask = distances < threshold
    refined_inliers = points[refined_inlier_mask]
    normal, d = fit_plane_svd(refined_inliers)
    return normal, d, refined_inliers


def sample_points(points: np.ndarray, sample_limit: int, seed: int):
    if len(points) <= sample_limit:
        return points
    rng = np.random.default_rng(seed)
    sample_idx = rng.choice(len(points), size=sample_limit, replace=False)
    return points[sample_idx]


def percentile_scan_values(min_percentile: float, max_percentile: float, steps: int):
    min_percentile = float(np.clip(min_percentile, 0.0, 100.0))
    max_percentile = float(np.clip(max_percentile, min_percentile, 100.0))
    steps = max(int(steps), 1)
    return np.unique(np.linspace(min_percentile, max_percentile, steps))


def refine_plane_with_full_support(xyz: np.ndarray, normal: np.ndarray, d: float, threshold: float):
    full_distances = plane_distances(xyz, normal, d)
    full_inlier_mask = full_distances < threshold
    full_inliers = xyz[full_inlier_mask]
    if len(full_inliers) >= 3:
        normal, d = fit_plane_svd(full_inliers)
        full_distances = plane_distances(xyz, normal, d)
        full_inlier_mask = full_distances < threshold
        full_inliers = xyz[full_inlier_mask]
    mean_distance = float(full_distances[full_inlier_mask].mean()) if len(full_inliers) else float("inf")
    return normal, d, full_inliers, mean_distance


def normalize_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def compute_ground_slopes(normal: np.ndarray):
    a = -normal[0] / normal[2]
    b = -normal[1] / normal[2]
    tilt_deg = math.degrees(math.acos(np.clip(normal[2], -1.0, 1.0)))
    slope_x_deg = math.degrees(math.atan(a))
    slope_y_deg = math.degrees(math.atan(b))
    return tilt_deg, slope_x_deg, slope_y_deg, a, b


def circular_mean_deg(angles_deg: np.ndarray, weights: np.ndarray = None, period_deg: float = 180.0) -> float:
    if weights is None:
        weights = np.ones(len(angles_deg), dtype=np.float64)
    radians = np.deg2rad(angles_deg * (360.0 / period_deg))
    vec = np.sum(weights * np.exp(1j * radians))
    if abs(vec) < 1e-12:
        raise RuntimeError("Circular mean is ill-defined for the detected directions")
    mean_deg = math.degrees(math.atan2(vec.imag, vec.real)) * (period_deg / 360.0)
    return normalize_periodic_angle_deg(mean_deg, period_deg)


def normalize_periodic_angle_deg(angle_deg: float, period_deg: float) -> float:
    half = period_deg / 2.0
    wrapped = (angle_deg + half) % period_deg - half
    if wrapped == -half:
        return half
    return wrapped


def nearest_axis_offset_deg(angle_deg: float) -> float:
    candidates = np.array([-90.0, 0.0, 90.0])
    diffs = np.array([normalize_angle_deg(angle_deg - c) for c in candidates])
    idx = int(np.argmin(np.abs(diffs)))
    return float(diffs[idx]), float(candidates[idx])


def rotation_matrix_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def rotation_matrix_from_vectors(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    src = src / np.linalg.norm(src)
    dst = dst / np.linalg.norm(dst)
    cross = np.cross(src, dst)
    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
    cross_norm = np.linalg.norm(cross)

    if cross_norm < 1e-12:
        if dot > 0:
            return np.eye(3)
        axis = np.array([1.0, 0.0, 0.0])
        if abs(src[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - src * np.dot(src, axis)
        axis /= np.linalg.norm(axis)
        return rotation_matrix_axis_angle(axis, math.pi)

    axis = cross / cross_norm
    angle = math.atan2(cross_norm, dot)
    return rotation_matrix_axis_angle(axis, angle)


def rotation_matrix_z(angle_deg: float) -> np.ndarray:
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def rotation_to_rpy_deg(rot: np.ndarray):
    pitch = math.asin(-np.clip(rot[2, 0], -1.0, 1.0))
    roll = math.atan2(rot[2, 1], rot[2, 2])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def estimate_ground(xyz: np.ndarray, args):
    z_values = xyz[:, 2]
    scan_percentiles = percentile_scan_values(
        args.ground_percentile,
        args.ground_max_percentile,
        args.ground_scan_steps,
    )
    best = None

    for percentile in scan_percentiles:
        z_threshold = np.percentile(z_values, percentile)
        candidates = xyz[z_values <= z_threshold]
        if len(candidates) < 3:
            continue

        sampled_candidates = sample_points(candidates, args.sample_limit, args.seed)
        normal, d, _ = ransac_plane(
            sampled_candidates,
            iterations=args.iterations,
            threshold=args.inlier_threshold,
            seed=args.seed,
        )
        normal, d, inliers, mean_distance = refine_plane_with_full_support(
            xyz,
            normal,
            d,
            args.inlier_threshold,
        )
        tilt_deg, slope_x_deg, slope_y_deg, a, b = compute_ground_slopes(normal)
        if tilt_deg > args.max_ground_tilt_deg:
            continue

        result = {
            "search_percentile_min": float(args.ground_percentile),
            "search_percentile_max": float(args.ground_max_percentile),
            "search_steps": int(args.ground_scan_steps),
            "selected_percentile": float(percentile),
            "z_threshold": float(z_threshold),
            "candidate_count": len(sampled_candidates),
            "candidate_pool_count": len(candidates),
            "normal": normal,
            "d": float(d),
            "inliers": inliers,
            "global_support_count": len(inliers),
            "mean_inlier_distance": float(mean_distance),
            "tilt_deg": float(tilt_deg),
            "slope_x_deg": float(slope_x_deg),
            "slope_y_deg": float(slope_y_deg),
            "plane_a": float(a),
            "plane_b": float(b),
        }
        if best is None:
            best = result
            continue

        best_key = (
            best["global_support_count"],
            -best["mean_inlier_distance"],
            -best["tilt_deg"],
            -best["selected_percentile"],
        )
        result_key = (
            result["global_support_count"],
            -result["mean_inlier_distance"],
            -result["tilt_deg"],
            -result["selected_percentile"],
        )
        if result_key > best_key:
            best = result

    if best is None:
        raise RuntimeError(
            "Failed to find a stable ground plane; try increasing --max-ground-tilt-deg "
            "or widening --ground-max-percentile"
        )

    normal = best["normal"]
    level_rotation = rotation_matrix_from_vectors(normal, np.array([0.0, 0.0, 1.0]))
    level_roll_deg, level_pitch_deg, level_yaw_deg = rotation_to_rpy_deg(level_rotation)

    return {
        **best,
        "level_rotation": level_rotation,
        "level_roll_deg": float(level_roll_deg),
        "level_pitch_deg": float(level_pitch_deg),
        "level_yaw_deg": float(level_yaw_deg),
    }


def print_ground_result(ground: dict, total_points: int):
    normal = ground["normal"]
    print("Ground analysis")
    print(f"Total valid points: {total_points}")
    print(
        f"Ground search percentiles: {ground['search_percentile_min']:.1f}% -> "
        f"{ground['search_percentile_max']:.1f}% in {ground['search_steps']} steps"
    )
    print(
        f"Selected ground percentile: {ground['selected_percentile']:.1f}% "
        f"(z <= {ground['z_threshold']:.4f} m), candidates used: {ground['candidate_count']} "
        f"from pool {ground['candidate_pool_count']}"
    )
    print(
        f"Plane normal: [{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}], "
        f"d = {ground['d']:.6f}"
    )
    print(f"Plane form: z = {ground['plane_a']:.6f} * x + {ground['plane_b']:.6f} * y + c")
    print(
        f"Ground support on full cloud: {ground['global_support_count']} points "
        f"(mean inlier distance {ground['mean_inlier_distance']:.4f} m)"
    )
    print(f"Estimated tilt from horizontal: {ground['tilt_deg']:.3f} deg")
    print(f"Slope along +x: {ground['slope_x_deg']:.3f} deg")
    print(f"Slope along +y: {ground['slope_y_deg']:.3f} deg")
    print(
        "Leveling rotation from ground: "
        f"roll={ground['level_roll_deg']:.3f} deg, "
        f"pitch={ground['level_pitch_deg']:.3f} deg, "
        f"yaw={ground['level_yaw_deg']:.3f} deg"
    )


def extract_wall_cells(xyz: np.ndarray, grid_size: float, min_z_span: float, min_cell_points: int) -> np.ndarray:
    xy = xyz[:, :2]
    z = xyz[:, 2]
    min_xy = xy.min(axis=0)
    grid_idx = np.floor((xy - min_xy) / grid_size).astype(np.int32)

    cells = {}
    for i, (ix, iy) in enumerate(grid_idx):
        key = (int(ix), int(iy))
        if key not in cells:
            cells[key] = [z[i], z[i], 1, xy[i].copy()]
        else:
            cell = cells[key]
            cell[0] = min(cell[0], z[i])
            cell[1] = max(cell[1], z[i])
            cell[2] += 1
            cell[3] += xy[i]

    wall_points = []
    for z_min, z_max, count, xy_sum in cells.values():
        if (z_max - z_min) >= min_z_span and count >= min_cell_points:
            wall_points.append(xy_sum / count)

    if not wall_points:
        raise RuntimeError("No wall-like vertical cells found; try lowering --min-z-span or --min-cell-points")

    return np.asarray(wall_points, dtype=np.float64)


def estimate_wall_yaw(wall_points: np.ndarray, args):
    points = wall_points
    if len(points) > args.sample_limit:
        rng = np.random.default_rng(args.seed)
        sample_idx = rng.choice(len(points), size=args.sample_limit, replace=False)
        points = points[sample_idx]

    tree = cKDTree(points)
    angles = []
    weights = []

    for point in points:
        neighbor_ids = tree.query_ball_point(point, args.neighbor_radius)
        if len(neighbor_ids) < args.min_neighbors:
            continue

        local = points[neighbor_ids] - point
        cov = local.T @ local / len(local)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]
        minor = max(float(eigvals[0]), 1e-12)
        major = float(eigvals[1])
        ratio = major / minor
        if ratio < args.linearity_ratio:
            continue

        direction = eigvecs[:, 1]
        angle_deg = math.degrees(math.atan2(direction[1], direction[0]))
        angle_deg = normalize_periodic_angle_deg(angle_deg, 180.0)
        angles.append(angle_deg)
        weights.append(ratio)

    if not angles:
        raise RuntimeError("No stable wall directions found; try increasing --neighbor-radius or lowering --linearity-ratio")

    angles = np.asarray(angles, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)

    bin_width = args.hist_bin_deg
    bins = np.arange(-90.0, 90.0 + bin_width, bin_width)
    hist, edges = np.histogram(angles, bins=bins, weights=weights)
    peak_idx = int(np.argmax(hist))
    center_deg = 0.5 * (edges[peak_idx] + edges[peak_idx + 1])
    half_window = max(1.5 * bin_width, 3.0)
    delta = np.array([normalize_periodic_angle_deg(a - center_deg, 180.0) for a in angles])
    peak_mask = np.abs(delta) <= half_window
    if not np.any(peak_mask):
        peak_mask = hist[peak_idx] > 0

    dominant_yaw_deg = circular_mean_deg(angles[peak_mask], weights[peak_mask], period_deg=180.0)
    offset_deg, nearest_axis_deg = nearest_axis_offset_deg(dominant_yaw_deg)

    orthogonal_yaw_deg = normalize_periodic_angle_deg(dominant_yaw_deg + 90.0, 180.0)
    ortho_offset_deg, ortho_axis_deg = nearest_axis_offset_deg(orthogonal_yaw_deg)
    if abs(ortho_offset_deg) < abs(offset_deg):
        dominant_yaw_deg = orthogonal_yaw_deg
        offset_deg = ortho_offset_deg
        nearest_axis_deg = ortho_axis_deg

    return {
        "wall_cells": len(wall_points),
        "sampled_wall_cells": len(points),
        "direction_samples": len(angles),
        "dominant_yaw_deg": dominant_yaw_deg,
        "axis_offset_deg": offset_deg,
        "nearest_axis_deg": nearest_axis_deg,
        "peak_window_deg": half_window,
    }


def estimate_wall_yaw_from_xyz(xyz: np.ndarray, args):
    wall_points = extract_wall_cells(
        xyz,
        grid_size=args.xy_grid,
        min_z_span=args.min_z_span,
        min_cell_points=args.min_cell_points,
    )
    return estimate_wall_yaw(wall_points, args)


def print_wall_result(wall: dict, total_points: int, leveled: bool, args):
    label = "Wall analysis on ground-leveled cloud" if leveled else "Wall analysis"
    print(label)
    print(f"Total valid points: {total_points}")
    print(
        f"Wall-like XY cells: {wall['wall_cells']} "
        f"(grid={args.xy_grid:.2f} m, min_z_span={args.min_z_span:.2f} m, min_cell_points={args.min_cell_points})"
    )
    print(f"Sampled wall cells for local PCA: {wall['sampled_wall_cells']}")
    print(f"Stable local wall directions used: {wall['direction_samples']}")
    print(
        "Dominant wall yaw around z: "
        f"{wall['dominant_yaw_deg']:.3f} deg "
        "(0 deg = +x, positive CCW toward +y, wall direction modulo 180 deg)"
    )
    print(
        f"Offset from nearest map axis ({wall['nearest_axis_deg']:.0f} deg): "
        f"{wall['axis_offset_deg']:.3f} deg"
    )
    print(
        "Interpretation: if this offset is close to 0 deg, the map is already 'square' with the XY axes; "
        "otherwise it is rotated around z by roughly that amount."
    )


def analyze_ground_mode(xyz: np.ndarray, args):
    print(f"PCD: {args.pcd}")
    print("Mode: ground")
    ground = estimate_ground(xyz, args)
    print_ground_result(ground, len(xyz))


def analyze_yaw_mode(xyz: np.ndarray, args):
    print(f"PCD: {args.pcd}")
    print("Mode: yaw")
    wall = estimate_wall_yaw_from_xyz(xyz, args)
    print_wall_result(wall, len(xyz), leveled=False, args=args)


def analyze_all_mode(xyz: np.ndarray, args):
    print(f"PCD: {args.pcd}")
    print("Mode: all")

    ground = estimate_ground(xyz, args)
    print_ground_result(ground, len(xyz))

    leveled_xyz = (ground["level_rotation"] @ xyz.T).T
    wall = estimate_wall_yaw_from_xyz(leveled_xyz, args)
    print_wall_result(wall, len(leveled_xyz), leveled=True, args=args)

    yaw_correction_deg = -wall["axis_offset_deg"]
    total_rotation = rotation_matrix_z(yaw_correction_deg) @ ground["level_rotation"]
    total_roll_deg, total_pitch_deg, total_yaw_deg = rotation_to_rpy_deg(total_rotation)

    print("Combined 3-axis estimate")
    print(
        "Suggested correction to apply to the map: "
        f"roll={total_roll_deg:.3f} deg, "
        f"pitch={total_pitch_deg:.3f} deg, "
        f"yaw={total_yaw_deg:.3f} deg"
    )
    print(
        "Source breakdown: "
        f"roll/pitch come from the ground plane; yaw comes from wall alignment after leveling."
    )
    print(
        "Yaw-only correction from walls after leveling: "
        f"{yaw_correction_deg:.3f} deg"
    )


def main():
    args = parse_args()
    xyz = read_pcd_xyz(args.pcd)
    if len(xyz) < 3:
        raise RuntimeError("Not enough valid points in the PCD")

    if args.mode == "ground":
        analyze_ground_mode(xyz, args)
    elif args.mode == "yaw":
        analyze_yaw_mode(xyz, args)
    else:
        analyze_all_mode(xyz, args)


if __name__ == "__main__":
    main()
