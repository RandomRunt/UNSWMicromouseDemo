from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree

# param.yaml area/length thresholds are written for ~this image width.
# Larger photos (e.g. maze13 @ 2914px) scale those pixel units up so cyan
# posts and outer-frame Hough lengths are not clipped. Never scales down —
# smaller captures already work with the loose headroom in the config.
DEFAULT_REF_WIDTH_PX = 1920


@dataclass
class Params:
    maze: dict
    image: dict
    output: dict
    poles: dict
    corners: dict
    warp: dict
    walls: dict
    shell: dict
    map: dict
    robot: dict
    path: dict
    planning: dict
    cylinders: dict
    enable_cylinder_detection: bool


@dataclass
class PoleDetection:
    poles_px: np.ndarray
    mask: np.ndarray
    overlay_bgr: np.ndarray


def load_params(path: str | Path) -> Params:
    config_path = Path(path)
    with open(config_path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return Params(
        maze=raw["maze"],
        image=raw["image"],
        output=raw["output"],
        poles=raw["poles"],
        corners=raw.get("corners", {}),
        warp=raw.get("warp", {}),
        walls=raw.get("walls", {}),
        shell=raw.get("shell", {}),
        map=raw.get("map", {}),
        robot=raw.get("robot", {}),
        path=raw.get("path", {}),
        planning=raw.get("planning", {}),
        cylinders=raw.get("cylinders", {}),
        enable_cylinder_detection=bool(raw.get("enable_cylinder_detection", False)),
    )


def chamfer_missing_poles(cols: int, rows: int) -> set[tuple[int, int]]:
    """The 3 grid poles skipped at each chamfered corner (12 total on 10×10)."""
    last_c = cols - 1
    last_r = rows - 1
    return {
        (0, 0),
        (0, 1),
        (1, 0),
        (last_c, 0),
        (last_c - 1, 0),
        (last_c, 1),
        (0, last_r),
        (0, last_r - 1),
        (1, last_r),
        (last_c, last_r),
        (last_c - 1, last_r),
        (last_c, last_r - 1),
    }


def image_pixel_scale(
    image_shape: tuple[int, ...],
    *,
    ref_width_px: float = DEFAULT_REF_WIDTH_PX,
) -> float:
    """Linear scale ≥ 1 relative to the reference capture width."""
    width = float(image_shape[1])
    ref = float(ref_width_px) if ref_width_px > 0 else float(DEFAULT_REF_WIDTH_PX)
    return max(1.0, width / ref)


def _scale_int(value: float | int, scale: float, *, minimum: int = 1) -> int:
    return max(minimum, int(round(float(value) * scale)))


def _force_odd(value: int, *, minimum: int = 3) -> int:
    v = max(minimum, int(value))
    if v % 2 == 0:
        v += 1
    return v


def scale_params_for_image(params: Params, image_shape: tuple[int, ...]) -> Params:
    """Copy of ``params`` with pixel thresholds enlarged for big photos.

    Area-like keys scale with ``s²``; length-like keys with ``s``. No-op when
    the image is at or below the reference width.
    """
    ref = float(params.poles.get("ref_width_px", DEFAULT_REF_WIDTH_PX))
    scale = image_pixel_scale(image_shape, ref_width_px=ref)
    if abs(scale - 1.0) < 1e-9:
        return params

    area_scale = scale * scale
    poles = dict(params.poles)
    for key in ("min_area", "max_area"):
        if key in poles:
            poles[key] = _scale_int(poles[key], area_scale, minimum=1)
    for key in (
        "max_blob_side",
        "merge_distance_px",
        "morph_kernel",
        "overlay_circle_radius",
        "frame_filter_margin_px",
    ):
        if key in poles:
            poles[key] = _scale_int(poles[key], scale, minimum=1)
    if "morph_kernel" in poles:
        poles["morph_kernel"] = _force_odd(int(poles["morph_kernel"]), minimum=3)

    corners = dict(params.corners)
    for key in (
        "outer_band_dilate_px",
        "outer_band_erode_px",
        "hough_min_length",
        "hough_max_gap",
        "outer_min_line_length",
        "overlay_circle_radius",
        "colour_morph_kernel",
    ):
        if key in corners:
            corners[key] = _scale_int(corners[key], scale, minimum=1)
    if "colour_min_area" in corners:
        corners["colour_min_area"] = _scale_int(
            corners["colour_min_area"], area_scale, minimum=1
        )
    if "colour_morph_kernel" in corners:
        corners["colour_morph_kernel"] = _force_odd(
            int(corners["colour_morph_kernel"]), minimum=3
        )

    cylinders = dict(params.cylinders)
    for key in ("morph_open", "morph_close"):
        if key in cylinders:
            cylinders[key] = _force_odd(
                _scale_int(cylinders[key], scale, minimum=1), minimum=1
            )

    return replace(params, poles=poles, corners=corners, cylinders=cylinders)


def _resolve_path(config_path: Path, rel: str) -> Path:
    p = Path(rel)
    if p.is_absolute():
        return p
    return (config_path.parent / p).resolve()


def _pole_mask(image_bgr: np.ndarray, poles: dict) -> np.ndarray:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(poles["hsv_lo"], dtype=np.uint8),
        np.array(poles["hsv_hi"], dtype=np.uint8),
    )
    k = int(poles["morph_kernel"])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _blob_centroids(mask: np.ndarray, poles: dict) -> np.ndarray:
    min_area = int(poles["min_area"])
    max_area = int(poles["max_area"])
    max_side = int(poles["max_blob_side"])
    max_aspect = float(poles["max_aspect"])

    n_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    points: list[tuple[float, float]] = []
    for i in range(1, n_labels):
        area = int(stats[i, cv2.CC_STAT_AREA])
        width = int(stats[i, cv2.CC_STAT_WIDTH])
        height = int(stats[i, cv2.CC_STAT_HEIGHT])
        if not (min_area <= area <= max_area):
            continue
        if width > max_side or height > max_side:
            continue
        short = max(1, min(width, height))
        if max(width, height) / short > max_aspect:
            continue
        cx, cy = centroids[i]
        points.append((float(cx), float(cy)))
    if not points:
        return np.zeros((0, 2), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _filter_by_local_density(points: np.ndarray, poles: dict) -> np.ndarray:
    min_neighbours = int(poles["min_neighbours"])
    if len(points) < min_neighbours:
        return points

    tree = cKDTree(points)
    nn_dists = np.array([tree.query(p, k=2)[0][1] for p in points], dtype=np.float64)
    median_nn = float(np.median(nn_dists))
    if median_nn <= 1e-3:
        return points

    radius = float(poles["neighbour_radius_scale"]) * median_nn
    keep = [
        p
        for p in points
        if len(tree.query_ball_point(p, r=radius)) >= min_neighbours
    ]
    if not keep:
        return np.zeros((0, 2), dtype=np.float32)
    return np.asarray(keep, dtype=np.float32)


def _merge_close_points(points: np.ndarray, poles: dict) -> np.ndarray:
    max_dist = float(poles.get("merge_distance_px", 0))
    if len(points) == 0 or max_dist <= 0:
        return points

    # Higher-res photos have larger NN; fixed merge_distance_px alone then
    # leaves half-cell false poles. Scale up to a fraction of median NN.
    if len(points) >= 4:
        tree0 = cKDTree(points)
        nn0 = np.array(
            [tree0.query(p, k=2)[0][1] for p in points], dtype=np.float64
        )
        med_nn = float(np.median(nn0))
        if med_nn > 1e-3:
            max_dist = max(max_dist, 0.45 * med_nn)

    n = len(points)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[rj] = ri

    tree = cKDTree(points)
    for i, j in tree.query_pairs(max_dist):
        union(i, j)

    clusters: dict[int, list[int]] = {}
    for i in range(n):
        clusters.setdefault(find(i), []).append(i)

    merged = [points[idxs].mean(axis=0) for idxs in clusters.values()]
    return np.asarray(merged, dtype=np.float32)


def _draw_overlay(image_bgr: np.ndarray, points: np.ndarray, poles: dict) -> np.ndarray:
    r = int(poles["overlay_circle_radius"])
    overlay = image_bgr.copy()
    for i, (x, y) in enumerate(points):
        cv2.circle(overlay, (int(round(x)), int(round(y))), r, (0, 0, 255), 2)
        cv2.putText(
            overlay,
            str(i),
            (int(round(x)) + 6, int(round(y)) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return overlay


def filter_poles_inside_frame(
    poles_px: np.ndarray,
    outer_corners_px: np.ndarray,
    *,
    margin_px: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep poles inside the outer-frame polygon (TL/TR/BR/BL).

    `margin_px` > 0 expands the quad so rails/near-edge poles survive;
    negative insets it. Returns (kept_poles, boolean mask over input).
    If the quad is degenerate (area≈0), keeps everything unchanged.
    """
    if len(poles_px) == 0:
        return poles_px.copy(), np.zeros((0,), dtype=bool)
    if outer_corners_px.shape != (4, 2):
        raise ValueError("outer_corners_px must be (4, 2)")

    poly = outer_corners_px.astype(np.float32)
    area = float(cv2.contourArea(poly))
    if area < 1.0:
        keep = np.ones(len(poles_px), dtype=bool)
        return poles_px.copy(), keep

    if margin_px != 0.0:
        centre = poly.mean(axis=0)
        dirs = poly - centre
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        norms = np.maximum(norms, 1e-6)
        poly = centre + dirs * (1.0 + margin_px / norms)

    contour = poly.reshape(-1, 1, 2)
    keep = np.array(
        [
            cv2.pointPolygonTest(contour, (float(x), float(y)), False) >= 0
            for x, y in poles_px
        ],
        dtype=bool,
    )
    return poles_px[keep].copy(), keep


def filter_pole_cluster(
    poles_px: np.ndarray,
    *,
    max_radius_spacings: float = 6.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Drop spatial outliers (e.g. shoes) far from the main pole cluster.

    Keeps poles within `max_radius_spacings * median_nearest_neighbour` of the
    coordinate-wise median centre.
    """
    if len(poles_px) < 4:
        keep = np.ones(len(poles_px), dtype=bool)
        return poles_px.copy(), keep

    tree = cKDTree(poles_px)
    nn = np.array(
        [tree.query(p, k=2)[0][1] for p in poles_px],
        dtype=np.float64,
    )
    spacing = float(np.median(nn))
    if spacing <= 1e-3:
        keep = np.ones(len(poles_px), dtype=bool)
        return poles_px.copy(), keep

    centre = np.median(poles_px, axis=0)
    dist = np.linalg.norm(poles_px - centre, axis=1)
    keep = dist <= max_radius_spacings * spacing
    if int(keep.sum()) < 4:
        keep = np.ones(len(poles_px), dtype=bool)
        return poles_px.copy(), keep
    return poles_px[keep].copy(), keep


def replace_pole_points(
    detection: PoleDetection,
    image_bgr: np.ndarray,
    poles_px: np.ndarray,
    params: Params,
) -> PoleDetection:
    """Rebuild a PoleDetection with a new point set (overlay/mask refreshed)."""
    points = np.asarray(poles_px, dtype=np.float32).reshape(-1, 2)
    if len(points):
        order = np.lexsort((points[:, 0], points[:, 1]))
        points = points[order]
    return PoleDetection(
        poles_px=points,
        mask=detection.mask,
        overlay_bgr=_draw_overlay(image_bgr, points, params.poles),
    )


def detect_poles(image_bgr: np.ndarray, params: Params) -> PoleDetection:
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("image_bgr is empty")

    poles = params.poles
    mask = _pole_mask(image_bgr, poles)
    raw = _blob_centroids(mask, poles)
    points = _filter_by_local_density(raw, poles)
    points = _merge_close_points(points, poles)
    if len(points):
        order = np.lexsort((points[:, 0], points[:, 1]))
        points = points[order]

    return PoleDetection(
        poles_px=points,
        mask=mask,
        overlay_bgr=_draw_overlay(image_bgr, points, poles),
    )


def pole_detection_from_points(
    image_bgr: np.ndarray,
    points_px: np.ndarray,
    params: Params,
    *,
    mask: np.ndarray | None = None,
) -> PoleDetection:
    """Build a PoleDetection (and overlay) from an explicit point set."""
    pts = np.asarray(points_px, dtype=np.float32).reshape(-1, 2)
    if len(pts):
        order = np.lexsort((pts[:, 0], pts[:, 1]))
        pts = pts[order]
    if mask is None:
        mask = np.zeros(image_bgr.shape[:2], dtype=np.uint8)
    return PoleDetection(
        poles_px=pts,
        mask=mask,
        overlay_bgr=_draw_overlay(image_bgr, pts, params.poles),
    )


def detect_poles_from_path(image_path: str | Path, params: Params) -> PoleDetection:
    path = Path(image_path)
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return detect_poles(image, params)


def save_poles_debug(
    detection: PoleDetection,
    output_dir: str | Path,
    *,
    prefix: str = "01_poles",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_mask.png"), detection.mask)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), detection.overlay_bgr)
    np.savetxt(
        out / f"{prefix}_xy.txt",
        detection.poles_px,
        fmt="%.3f",
        header="x_px y_px",
        comments="",
    )
