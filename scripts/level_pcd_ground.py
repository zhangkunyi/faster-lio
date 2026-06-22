#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def parse_args():
    default_input = Path(__file__).resolve().parents[1] / "PCD" / "map_for_localization.pcd"
    parser = argparse.ArgumentParser(
        description=(
            "Align a PCD using the same logic as analyze_pcd_ground_tilt.py: "
            "ground plane for roll/pitch, walls for yaw."
        )
    )
    parser.add_argument(
        "pcd",
        nargs="?",
        default=default_input,
        type=Path,
        help="Path to the source PCD file.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output PCD path. Defaults to <input>_aligned.pcd",
    )
    parser.add_argument(
        "--ground-percentile",
        type=float,
        default=10.0,
        help="Use points below this z percentile as ground candidates. Default: 10",
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
        help="RANSAC iterations for ground fitting. Default: 400",
    )
    parser.add_argument(
        "--inlier-threshold",
        type=float,
        default=0.08,
        help="Ground plane inlier distance threshold in meters. Default: 0.08",
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
        help="XY grid size in meters for vertical-structure extraction. Default: 0.2",
    )
    parser.add_argument(
        "--min-z-span",
        type=float,
        default=1.5,
        help="Minimum z span per XY cell to be treated as wall-like. Default: 1.5",
    )
    parser.add_argument(
        "--min-cell-points",
        type=int,
        default=3,
        help="Minimum points in an XY cell. Default: 3",
    )
    parser.add_argument(
        "--neighbor-radius",
        type=float,
        default=0.8,
        help="Neighbor radius in meters for local direction fitting. Default: 0.8",
    )
    parser.add_argument(
        "--min-neighbors",
        type=int,
        default=5,
        help="Minimum nearby wall cells for local PCA. Default: 5",
    )
    parser.add_argument(
        "--linearity-ratio",
        type=float,
        default=4.0,
        help="Require major/minor eigenvalue ratio above this value. Default: 4.0",
    )
    parser.add_argument(
        "--hist-bin-deg",
        type=float,
        default=1.0,
        help="Histogram bin width in degrees over [-90, 90). Default: 1.0",
    )
    parser.add_argument(
        "--zero-ground",
        action="store_true",
        help="After alignment, shift the cloud along z so the fitted ground plane lies on z=0.",
    )
    return parser.parse_args()


def parse_pcd_header(path: Path):
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

    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        if count != 1:
            raise ValueError(f"Unsupported field count for {name}: {count}")
        if typ == "F" and size == 4:
            dtype_fields.append((name, "<f4"))
        elif typ == "F" and size == 8:
            dtype_fields.append((name, "<f8"))
        elif typ == "I" and size == 1:
            dtype_fields.append((name, "i1"))
        elif typ == "I" and size == 2:
            dtype_fields.append((name, "<i2"))
        elif typ == "I" and size == 4:
            dtype_fields.append((name, "<i4"))
        elif typ == "U" and size == 1:
            dtype_fields.append((name, "u1"))
        elif typ == "U" and size == 2:
            dtype_fields.append((name, "<u2"))
        elif typ == "U" and size == 4:
            dtype_fields.append((name, "<u4"))
        else:
            raise ValueError(f"Unsupported field format: {name} {typ}{size}")

    return {
        "header_lines": header_lines,
        "fields": fields,
        "dtype": np.dtype(dtype_fields),
        "points": points,
        "data_kind": data_kind,
        "data_offset": data_offset,
    }


def read_pcd(path: Path):
    info = parse_pcd_header(path)
    if info["data_kind"] != "binary":
        raise ValueError("Only binary PCD input is supported")

    raw = np.fromfile(
        path,
        dtype=info["dtype"],
        offset=info["data_offset"],
        count=info["points"],
    )
    xyz = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float64, copy=False)
    return info, raw, xyz


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


def normalize_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def normalize_periodic_angle_deg(angle_deg: float, period_deg: float) -> float:
    half = period_deg / 2.0
    wrapped = (angle_deg + half) % period_deg - half
    if wrapped == -half:
        return half
    return wrapped


def circular_mean_deg(angles_deg: np.ndarray, weights: np.ndarray = None, period_deg: float = 180.0) -> float:
    if weights is None:
        weights = np.ones(len(angles_deg), dtype=np.float64)
    radians = np.deg2rad(angles_deg * (360.0 / period_deg))
    vec = np.sum(weights * np.exp(1j * radians))
    if abs(vec) < 1e-12:
        raise RuntimeError("Circular mean is ill-defined for the detected directions")
    mean_deg = math.degrees(math.atan2(vec.imag, vec.real)) * (period_deg / 360.0)
    return normalize_periodic_angle_deg(mean_deg, period_deg)


def nearest_axis_offset_deg(angle_deg: float):
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
    valid_xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(valid_xyz) < 3:
        raise RuntimeError("Not enough valid points in the PCD")

    z_values = valid_xyz[:, 2]
    z_threshold = np.percentile(z_values, args.ground_percentile)
    candidates = valid_xyz[z_values <= z_threshold]
    if len(candidates) < 3:
        raise RuntimeError("Not enough ground candidate points after z filtering")

    if len(candidates) > args.sample_limit:
        rng = np.random.default_rng(args.seed)
        sample_idx = rng.choice(len(candidates), size=args.sample_limit, replace=False)
        candidates = candidates[sample_idx]

    normal, d, inliers = ransac_plane(
        candidates,
        iterations=args.iterations,
        threshold=args.inlier_threshold,
        seed=args.seed,
    )
    level_rotation = rotation_matrix_from_vectors(normal, np.array([0.0, 0.0, 1.0]))
    return {
        "z_threshold": float(z_threshold),
        "candidate_count": len(candidates),
        "normal": normal,
        "d": float(d),
        "inliers": inliers,
        "level_rotation": level_rotation,
    }


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


def estimate_wall_yaw_from_xyz(xyz: np.ndarray, args):
    wall_points = extract_wall_cells(
        xyz,
        grid_size=args.xy_grid,
        min_z_span=args.min_z_span,
        min_cell_points=args.min_cell_points,
    )

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
        "dominant_yaw_deg": float(dominant_yaw_deg),
        "axis_offset_deg": float(offset_deg),
        "nearest_axis_deg": float(nearest_axis_deg),
    }


def update_header_lines(header_lines, point_count):
    updated = []
    for line in header_lines:
        if line.startswith("WIDTH "):
            updated.append(f"WIDTH {point_count}")
        elif line.startswith("HEIGHT "):
            updated.append("HEIGHT 1")
        elif line.startswith("POINTS "):
            updated.append(f"POINTS {point_count}")
        elif line.startswith("DATA "):
            updated.append("DATA binary")
        else:
            updated.append(line)
    return updated


def write_binary_pcd(path: Path, header_lines, array: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    updated_header = update_header_lines(header_lines, len(array))
    with path.open("wb") as f:
        for line in updated_header:
            f.write((line + "\n").encode("ascii"))
        array.tofile(f)


def main():
    args = parse_args()
    output_path = args.output or args.pcd.with_name(args.pcd.stem + "_aligned.pcd")

    info, raw, xyz = read_pcd(args.pcd)
    finite_mask = np.isfinite(xyz).all(axis=1)
    valid_xyz = xyz[finite_mask]
    if len(valid_xyz) < 3:
        raise RuntimeError("Not enough valid points in the PCD")

    ground = estimate_ground(valid_xyz, args)
    level_rotation = ground["level_rotation"]
    leveled_xyz = (level_rotation @ valid_xyz.T).T

    wall = estimate_wall_yaw_from_xyz(leveled_xyz, args)
    yaw_correction_deg = -wall["axis_offset_deg"]
    yaw_rotation = rotation_matrix_z(yaw_correction_deg)
    total_rotation = yaw_rotation @ level_rotation

    rotated_xyz = (total_rotation @ xyz.T).T
    z_shift = 0.0
    if args.zero_ground:
        rotated_ground_inliers = (total_rotation @ ground["inliers"].T).T
        z_shift = float(rotated_ground_inliers[:, 2].mean())
        rotated_xyz[:, 2] -= z_shift

    aligned = raw.copy()
    aligned["x"] = rotated_xyz[:, 0].astype(aligned["x"].dtype, copy=False)
    aligned["y"] = rotated_xyz[:, 1].astype(aligned["y"].dtype, copy=False)
    aligned["z"] = rotated_xyz[:, 2].astype(aligned["z"].dtype, copy=False)

    normal_fields = {"normal_x", "normal_y", "normal_z"}
    if normal_fields.issubset(info["fields"]):
        normals = np.column_stack([raw["normal_x"], raw["normal_y"], raw["normal_z"]]).astype(np.float64, copy=False)
        rotated_normals = (total_rotation @ normals.T).T
        aligned["normal_x"] = rotated_normals[:, 0].astype(aligned["normal_x"].dtype, copy=False)
        aligned["normal_y"] = rotated_normals[:, 1].astype(aligned["normal_y"].dtype, copy=False)
        aligned["normal_z"] = rotated_normals[:, 2].astype(aligned["normal_z"].dtype, copy=False)

    write_binary_pcd(output_path, info["header_lines"], aligned)

    roll_deg, pitch_deg, yaw_deg = rotation_to_rpy_deg(total_rotation)
    level_roll_deg, level_pitch_deg, level_yaw_deg = rotation_to_rpy_deg(level_rotation)

    print(f"Input PCD: {args.pcd}")
    print(f"Output PCD: {output_path}")
    print(f"Total points: {len(raw)}")
    print(
        f"Ground candidate percentile: {args.ground_percentile:.1f}% "
        f"(z <= {ground['z_threshold']:.4f} m), candidates used: {ground['candidate_count']}"
    )
    print(f"Ground inliers: {len(ground['inliers'])}")
    print(f"Ground normal before alignment: [{ground['normal'][0]:.6f}, {ground['normal'][1]:.6f}, {ground['normal'][2]:.6f}]")
    print(
        f"Wall yaw after ground leveling: {wall['dominant_yaw_deg']:.3f} deg, "
        f"axis offset: {wall['axis_offset_deg']:.3f} deg"
    )
    print(
        "Applied ground leveling rotation: "
        f"roll={level_roll_deg:.3f} deg, pitch={level_pitch_deg:.3f} deg, yaw={level_yaw_deg:.3f} deg"
    )
    print(f"Applied wall-yaw correction after leveling: {yaw_correction_deg:.3f} deg")
    print(
        "Applied total correction: "
        f"roll={roll_deg:.3f} deg, pitch={pitch_deg:.3f} deg, yaw={yaw_deg:.3f} deg"
    )
    if args.zero_ground:
        print(f"Applied z shift after alignment: {-z_shift:.6f} m")
    else:
        print("Applied z shift after alignment: 0.000000 m")


if __name__ == "__main__":
    main()
