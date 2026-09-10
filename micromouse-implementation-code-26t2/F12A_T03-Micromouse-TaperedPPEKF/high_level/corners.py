from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import ConvexHull

from poles import Params


@dataclass
class CornerDetection:
    outer_corners_px: np.ndarray
    diagonal_endpoints_px: np.ndarray
    colour_centroids_px: dict[str, np.ndarray]
    overlay_bgr: np.ndarray


def _seg_line(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float, float]:
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    return a, b, c


def _intersect(
    s1: tuple[float, float, float, float],
    s2: tuple[float, float, float, float],
) -> np.ndarray | None:
    a1, b1, c1 = _seg_line(*s1)
    a2, b2, c2 = _seg_line(*s2)
    d = a1 * b2 - a2 * b1
    if abs(d) < 1e-6:
        return None
    x = (b1 * c2 - b2 * c1) / d
    y = (c1 * a2 - c2 * a1) / d
    return np.array([x, y], dtype=np.float32)


def _hull_mask(points: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    hull = ConvexHull(points)
    pts = points[hull.vertices].astype(np.int32)
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, pts, 255)
    return mask


def _outer_band(hull: np.ndarray, dilate_px: int, erode_px: int) -> np.ndarray:
    k_d = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_px, dilate_px))
    k_e = cv2.getStructuringElement(cv2.MORPH_RECT, (erode_px, erode_px))
    return cv2.subtract(cv2.dilate(hull, k_d), cv2.erode(hull, k_e))


def _classify_lines(
    lines: np.ndarray,
    min_length: float,
) -> tuple[list, list]:
    horiz: list = []
    vert: list = []
    for l in lines[:, 0]:
        x1, y1, x2, y2 = map(float, l)
        ang = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180
        length = float(np.hypot(x2 - x1, y2 - y1))
        if length < min_length:
            continue
        midx = (x1 + x2) / 2
        midy = (y1 + y2) / 2
        seg = (midx, midy, x1, y1, x2, y2, length)
        if ang < 20 or ang > 160:
            horiz.append(seg)
        elif 70 < ang < 110:
            vert.append(seg)
    return horiz, vert


def _pick_extreme_rail(
    segs: list,
    *,
    coord: int,
    want_max: bool,
    prefer_length: float,
) -> tuple:
    """Pick the top/bottom/left/right frame rail among Hough segments.

    Takes the geometric extreme along ``coord`` (0=midx, 1=midy), then among
    segments near that extreme prefers ones that meet ``prefer_length``, else
    the longest. Avoids the failure mode where a short true top rail is
    filtered out and both "top" and "bot" snap to the bottom extrusion.
    """
    if not segs:
        raise RuntimeError("no Hough rails to pick from")

    mids = [float(s[coord]) for s in segs]
    extreme = max(mids) if want_max else min(mids)
    span = float(max(mids) - min(mids))
    tol = max(12.0, 0.08 * span) if span > 1.0 else 12.0
    near = [s for s in segs if abs(float(s[coord]) - extreme) <= tol]
    good = [s for s in near if float(s[6]) >= prefer_length]
    pool = good or near
    return max(pool, key=lambda s: float(s[6]))


def _detect_outer_corners(
    image_bgr: np.ndarray,
    poles_px: np.ndarray,
    corners_cfg: dict,
) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    hull = _hull_mask(poles_px, gray.shape)
    band = _outer_band(
        hull,
        int(corners_cfg["outer_band_dilate_px"]),
        int(corners_cfg["outer_band_erode_px"]),
    )
    edges = cv2.Canny(
        gray,
        int(corners_cfg["canny_lo"]),
        int(corners_cfg["canny_hi"]),
    )
    edges = cv2.bitwise_and(edges, band)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=int(corners_cfg["hough_threshold"]),
        minLineLength=int(corners_cfg["hough_min_length"]),
        maxLineGap=int(corners_cfg["hough_max_gap"]),
    )
    if lines is None:
        raise RuntimeError("No outer-frame lines found")

    # Soft length for classification (= Hough floor); prefer_length still
    # rewards long rails when several sit near the same extreme.
    soft_min = float(corners_cfg["hough_min_length"])
    prefer_len = float(corners_cfg["outer_min_line_length"])
    horiz, vert = _classify_lines(lines, soft_min)
    if not horiz or not vert:
        raise RuntimeError("Could not classify outer H/V frame lines")

    top = _pick_extreme_rail(horiz, coord=1, want_max=False, prefer_length=prefer_len)
    bot = _pick_extreme_rail(horiz, coord=1, want_max=True, prefer_length=prefer_len)
    left = _pick_extreme_rail(vert, coord=0, want_max=False, prefer_length=prefer_len)
    right = _pick_extreme_rail(vert, coord=0, want_max=True, prefer_length=prefer_len)

    segs = {
        "top": (top[2], top[3], top[4], top[5]),
        "bot": (bot[2], bot[3], bot[4], bot[5]),
        "left": (left[2], left[3], left[4], left[5]),
        "right": (right[2], right[3], right[4], right[5]),
    }
    order = [
        ("top", "left"),
        ("top", "right"),
        ("bot", "right"),
        ("bot", "left"),
    ]
    corners = []
    for a, b in order:
        p = _intersect(segs[a], segs[b])
        if p is None:
            raise RuntimeError(f"Failed to intersect {a}/{b}")
        corners.append(p)
    corners_arr = np.asarray(corners, dtype=np.float32)

    # Sanity: collapsed rails (both H near the bottom) throw, don't silently warp.
    pole_h = float(np.ptp(poles_px[:, 1])) if len(poles_px) else 0.0
    quad_h = float(np.ptp(corners_arr[:, 1]))
    if pole_h > 50.0 and quad_h < 0.35 * pole_h:
        raise RuntimeError(
            f"outer-frame quad collapsed (height {quad_h:.0f}px vs pole span {pole_h:.0f}px)"
        )
    return corners_arr



def _blob_axis_endpoints(mask: np.ndarray, label: int) -> tuple[np.ndarray, np.ndarray]:
    ys, xs = np.where(mask == label)
    pts = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
    mean = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - mean, full_matrices=False)
    direction = vt[0]
    proj = (pts - mean) @ direction
    p0 = mean + direction * float(proj.min())
    p1 = mean + direction * float(proj.max())
    return p0.astype(np.float32), p1.astype(np.float32)


def _detect_diagonal_colours(
    image_bgr: np.ndarray,
    corners_cfg: dict,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    k = int(corners_cfg["colour_morph_kernel"])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    min_area = int(corners_cfg["colour_min_area"])

    markers = corners_cfg["diagonal_colours"]
    endpoints: list[np.ndarray] = []
    centroids: dict[str, np.ndarray] = {}
    for name, spec in markers.items():
        lo = np.array(spec["hsv_lo"], dtype=np.uint8)
        hi = np.array(spec["hsv_hi"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        n, labels, stats, cens = cv2.connectedComponentsWithStats(mask)
        best_i, best_a = None, 0
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= min_area and area > best_a:
                best_a = area
                best_i = i
        if best_i is None:
            continue
        p0, p1 = _blob_axis_endpoints(labels, best_i)
        endpoints.extend([p0, p1])
        centroids[name] = np.array(
            [float(cens[best_i][0]), float(cens[best_i][1])],
            dtype=np.float32,
        )

    if not endpoints:
        return np.zeros((0, 2), dtype=np.float32), centroids
    return np.asarray(endpoints, dtype=np.float32), centroids


def _draw_overlay(
    image_bgr: np.ndarray,
    outer: np.ndarray,
    diagonals: np.ndarray,
    radius: int,
) -> np.ndarray:
    overlay = image_bgr.copy()
    labels = ["TL", "TR", "BR", "BL"]
    if len(outer) == 4:
        for i in range(4):
            a = outer[i]
            b = outer[(i + 1) % 4]
            cv2.line(
                overlay,
                (int(round(a[0])), int(round(a[1]))),
                (int(round(b[0])), int(round(b[1]))),
                (0, 255, 0),
                2,
            )
        for i, (x, y) in enumerate(outer):
            cv2.circle(overlay, (int(round(x)), int(round(y))), radius, (0, 0, 255), 3)
            cv2.putText(
                overlay,
                labels[i],
                (int(round(x)) + 10, int(round(y)) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
    for i, (x, y) in enumerate(diagonals):
        cv2.circle(overlay, (int(round(x)), int(round(y))), radius - 2, (255, 0, 255), 2)
        cv2.putText(
            overlay,
            f"D{i}",
            (int(round(x)) + 6, int(round(y)) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay


def detect_corners(
    image_bgr: np.ndarray,
    poles_px: np.ndarray,
    params: Params,
) -> CornerDetection:
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("image_bgr is empty")
    if len(poles_px) < 4:
        raise ValueError("need at least 4 poles to locate the outer frame")

    cfg = params.corners
    outer = _detect_outer_corners(image_bgr, poles_px, cfg)
    diagonals, colour_centroids = _detect_diagonal_colours(image_bgr, cfg)
    overlay = _draw_overlay(
        image_bgr,
        outer,
        diagonals,
        int(cfg["overlay_circle_radius"]),
    )
    return CornerDetection(
        outer_corners_px=outer,
        diagonal_endpoints_px=diagonals,
        colour_centroids_px=colour_centroids,
        overlay_bgr=overlay,
    )


def save_corners_debug(
    detection: CornerDetection,
    output_dir: str | Path,
    *,
    prefix: str = "02_corners",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), detection.overlay_bgr)
    np.savetxt(
        out / f"{prefix}_outer_xy.txt",
        detection.outer_corners_px,
        fmt="%.3f",
        header="x_px y_px  (TL TR BR BL)",
        comments="",
    )
    np.savetxt(
        out / f"{prefix}_diagonal_xy.txt",
        detection.diagonal_endpoints_px,
        fmt="%.3f",
        header="x_px y_px  (diagonal strip endpoints)",
        comments="",
    )
