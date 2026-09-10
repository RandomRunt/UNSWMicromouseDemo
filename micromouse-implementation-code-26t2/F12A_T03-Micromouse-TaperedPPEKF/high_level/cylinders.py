from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree

from poles import Params
from warp import FineWarpResult, project_points


@dataclass
class Cylinder:
    """One free-floating cylinder obstacle in maze millimetres."""

    x_mm: float
    y_mm: float
    radius_mm: float
    src_xy: tuple[float, float]
    warp_xy: tuple[float, float]
    radius_warp_px: float
    circularity: float
    fill: float


@dataclass
class CylinderDetection:
    cylinders: list[Cylinder]
    mask: np.ndarray
    overlay_bgr: np.ndarray
    overlay_warp_bgr: np.ndarray


def _cfg(params: Params) -> dict:
    return params.cylinders if getattr(params, "cylinders", None) else {}


def _expected_radius_warp_px(fine: FineWarpResult, params: Params) -> float:
    diam = float(_cfg(params).get("diameter_mm", 50.0))
    return 0.5 * diam * float(fine.pixels_per_mm)


def _dark_mask(gray: np.ndarray, cfg: dict) -> np.ndarray:
    thr = int(cfg.get("dark_threshold", 70))
    _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
    open_k = int(cfg.get("morph_open", 5))
    close_k = int(cfg.get("morph_close", 5))
    if open_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    if close_k > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def _maze_roi_mask(shape_hw: tuple[int, int], fine: FineWarpResult, params: Params) -> np.ndarray:
    """Keep detections inside the pole lattice (+margin), drop outer aluminium."""
    h, w = shape_hw
    roi = np.zeros((h, w), dtype=np.uint8)
    maze = params.maze
    fine_cfg = params.warp["fine"]
    margin = float(fine_cfg["margin_cells"])
    ppc = float(fine.px_per_cell)
    # Soft inset (positive = shrink) to ignore outer-frame / colour-marker blobs.
    inset = float(_cfg(params).get("roi_inset_cells", 0.15))
    cols = int(maze["pole_grid_cols"])
    rows = int(maze["pole_grid_rows"])
    x0 = (margin + inset) * ppc
    y0 = (margin + inset) * ppc
    x1 = (margin + (cols - 1) - inset) * ppc
    y1 = (margin + (rows - 1) - inset) * ppc
    pts = np.array(
        [
            [x0, y0],
            [x1, y0],
            [x1, y1],
            [x0, y1],
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(roi, pts, 255)
    return roi


def _inside_maze_mm(x_mm: float, y_mm: float, params: Params) -> bool:
    """Cylinders are free-floating but must still sit on the board."""
    cell = float(params.maze["cell_size_mm"])
    cols = int(params.maze["pole_grid_cols"])
    rows = int(params.maze["pole_grid_rows"])
    pad = float(_cfg(params).get("board_margin_mm", 0.5 * cell))
    xmin, ymin = -pad, -pad
    xmax = (cols - 1) * cell + pad
    ymax = (rows - 1) * cell + pad
    return xmin <= x_mm <= xmax and ymin <= y_mm <= ymax

def _blob_candidates(
    mask: np.ndarray,
    gray: np.ndarray,
    r_expect: float,
    cfg: dict,
) -> list[tuple[float, float, float, float, float]]:
    """Return (cx, cy, r_px, circularity, fill) via distance-transform peaks.

    Thin acrylic walls are severed by opening; cylinder cores remain as round
    DT peaks. Scoring uses a *fixed circular disk* around each peak (not a
    connected component) so touching / clustered cylinders still pass — CC
    contours merge neighbours and kill circularity.
    """
    # min_circularity kept for config compatibility; disk mask_fill is the
    # roundness proxy (1 = expected cylinder disk fully dark).
    min_circ = float(cfg.get("min_circularity", 0.70))
    min_fill = float(cfg.get("min_fill", cfg.get("min_mask_fill", 0.55)))
    r_lo = float(cfg.get("radius_scale_lo", 0.75)) * r_expect
    r_hi = float(cfg.get("radius_scale_hi", 2.40)) * r_expect
    max_mean = float(cfg.get("max_inner_mean", 55.0))
    min_contrast = float(cfg.get("min_ring_contrast", 40.0))
    disk_scale = float(cfg.get("disk_fill_radius_scale", 0.90))

    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    ksz = max(3, int(round(2.0 * r_expect)) | 1)
    local_max = cv2.dilate(
        dist, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
    )
    peaks = (dist == local_max) & (dist >= r_lo) & (dist <= r_hi)
    ys, xs = np.where(peaks)

    out: list[tuple[float, float, float, float, float]] = []
    h, w = gray.shape
    for x, y in zip(xs.astype(float), ys.astype(float)):
        yi, xi = int(y), int(x)
        peak = float(dist[yi, xi])
        if peak < 1.0:
            continue
        if float(gray[yi, xi]) > max_mean:
            continue

        ri = max(1, int(round(peak)))
        pad = max(ri * 2, ri + 4)
        y0, y1 = max(0, yi - pad), min(h, yi + pad + 1)
        x0, x1 = max(0, xi - pad), min(w, xi + pad + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        d2 = (xx - xi) ** 2 + (yy - yi) ** 2
        disk = d2 <= (disk_scale * ri) ** 2
        if int(disk.sum()) < 8:
            continue

        local_mask = mask[y0:y1, x0:x1]
        mask_fill = float((local_mask[disk] > 0).mean())
        # Treat fill as circularity proxy for NMS ranking / debug yaml.
        circ = mask_fill
        if circ < min_circ or mask_fill < min_fill:
            continue

        patch = gray[y0:y1, x0:x1]
        inner = patch[d2 <= (0.55 * ri) ** 2]
        ring = patch[(d2 > (1.15 * ri) ** 2) & (d2 < (1.70 * ri) ** 2)]
        if inner.size < 8 or ring.size < 8:
            continue
        mi = float(inner.mean())
        mr = float(ring.mean())
        if mi > max_mean or (mr - mi) < min_contrast:
            continue

        out.append((float(x), float(y), float(peak), circ, mask_fill))
    return out


def _nms(
    cands: list[tuple[float, float, float, float, float]],
    min_dist_px: float,
) -> list[tuple[float, float, float, float, float]]:
    ordered = sorted(cands, key=lambda t: (-t[3], -t[4], -t[2]))
    kept: list[tuple[float, float, float, float, float]] = []
    for c in ordered:
        if any((c[0] - k[0]) ** 2 + (c[1] - k[1]) ** 2 < min_dist_px**2 for k in kept):
            continue
        kept.append(c)
    return kept


def _warp_to_mm(
    x_px: float,
    y_px: float,
    fine: FineWarpResult,
    params: Params,
) -> tuple[float, float]:
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    cell = float(params.maze["cell_size_mm"])
    x_mm = (x_px / ppc - margin) * cell
    y_mm = (y_px / ppc - margin) * cell
    return float(x_mm), float(y_mm)


def _mm_to_src_via_inverse(
    centres_warp: np.ndarray,
    fine: FineWarpResult,
) -> np.ndarray:
    """Map fine-warp pixel centres back to source image pixels."""
    if len(centres_warp) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    inv = np.linalg.inv(fine.matrix.astype(np.float64))
    return project_points(centres_warp.astype(np.float32), inv.astype(np.float32))


def detect_cylinders(
    image_bgr: np.ndarray,
    fine: FineWarpResult,
    params: Params,
) -> CylinderDetection:
    """Detect black PLA cylinders on the fine-warped maze view.

    Centres are free-floating (not snapped to the pole lattice). Occupancy uses
    `cylinders.diameter_mm`; detection allows a looser warp-pixel radius band
    because overhead shadows inflate the dark disk.
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("image_bgr is empty")
    cfg = _cfg(params)
    warped = fine.warped_bgr
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    r_expect = _expected_radius_warp_px(fine, params)
    mask = _dark_mask(gray, cfg)
    roi = _maze_roi_mask(gray.shape, fine, params)
    mask = cv2.bitwise_and(mask, roi)

    cands = _blob_candidates(mask, gray, r_expect, cfg)
    min_sep = float(cfg.get("nms_radius_scale", 1.3)) * r_expect
    cands = _nms(cands, min_sep)

    diam_mm = float(cfg.get("diameter_mm", 50.0))
    radius_mm = 0.5 * diam_mm

    warp_xy = np.array([[c[0], c[1]] for c in cands], dtype=np.float32)
    src_xy = _mm_to_src_via_inverse(warp_xy, fine)

    cylinders: list[Cylinder] = []
    for i, (cx, cy, r_px, circ, fill) in enumerate(cands):
        x_mm, y_mm = _warp_to_mm(cx, cy, fine, params)
        if not _inside_maze_mm(x_mm, y_mm, params):
            continue
        sx, sy = (float(src_xy[i, 0]), float(src_xy[i, 1])) if len(src_xy) else (0.0, 0.0)
        cylinders.append(
            Cylinder(
                x_mm=x_mm,
                y_mm=y_mm,
                radius_mm=radius_mm,
                src_xy=(sx, sy),
                warp_xy=(float(cx), float(cy)),
                radius_warp_px=float(r_px),
                circularity=float(circ),
                fill=float(fill),
            )
        )
    # Stable order for debug / diffs.
    cylinders.sort(key=lambda c: (c.y_mm, c.x_mm))

    overlay_warp = warped.copy()
    overlay_src = image_bgr.copy()
    for cyl in cylinders:
        wx, wy = cyl.warp_xy
        cv2.circle(
            overlay_warp,
            (int(round(wx)), int(round(wy))),
            max(1, int(round(r_expect))),
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.circle(
            overlay_warp,
            (int(round(wx)), int(round(wy))),
            2,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )
        sx, sy = cyl.src_xy
        cv2.circle(
            overlay_src,
            (int(round(sx)), int(round(sy))),
            max(4, int(round(cyl.radius_warp_px * 0.7))),
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.circle(
            overlay_src,
            (int(round(sx)), int(round(sy))),
            3,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )

    return CylinderDetection(
        cylinders=cylinders,
        mask=mask,
        overlay_bgr=overlay_src,
        overlay_warp_bgr=overlay_warp,
    )


@dataclass
class CylinderRegionCells:
    """Axis-aligned block of maze cells covering the cylinder section."""

    col0: int
    row0: int
    width: int
    height: int

    @property
    def col1(self) -> int:
        return self.col0 + self.width - 1

    @property
    def row1(self) -> int:
        return self.row0 + self.height - 1

    def cells(self) -> list[tuple[int, int]]:
        return [
            (c, r)
            for r in range(self.row0, self.row0 + self.height)
            for c in range(self.col0, self.col0 + self.width)
        ]


def _mm_to_fine_px(
    x_mm: float,
    y_mm: float,
    fine: FineWarpResult,
    params: Params,
) -> tuple[int, int]:
    maze_cell = float(params.maze["cell_size_mm"])
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    return (
        int(round((x_mm / maze_cell + margin) * ppc)),
        int(round((y_mm / maze_cell + margin) * ppc)),
    )


def _cylinder_center_aabb(
    cylinders: list[Cylinder],
    *,
    cell_mm: float,
    grid_cols: int,
    grid_rows: int,
) -> tuple[int, int, int, int] | None:
    """Inclusive (col0, row0, col1, row1) of cells containing cylinder centres.

    Disk extent is intentionally ignored: a centre near a cell edge can spill a
    few cm of radius into a neighbour and inflate the seed past the real open
    chamber (e.g. 6×1 instead of 5×5).
    """
    if not cylinders:
        return None
    c0 = grid_cols
    r0 = grid_rows
    c1 = -1
    r1 = -1
    for cyl in cylinders:
        cc = int(np.floor(float(cyl.x_mm) / cell_mm))
        rr = int(np.floor(float(cyl.y_mm) / cell_mm))
        c0 = min(c0, cc)
        r0 = min(r0, rr)
        c1 = max(c1, cc)
        r1 = max(r1, rr)
    c0 = max(0, min(c0, grid_cols - 1))
    r0 = max(0, min(r0, grid_rows - 1))
    c1 = max(0, min(c1, grid_cols - 1))
    r1 = max(0, min(r1, grid_rows - 1))
    if c1 < c0 or r1 < r0:
        return None
    return c0, r0, c1, r1


def _grow_aabb_until_walls(
    c0: int,
    r0: int,
    c1: int,
    r1: int,
    *,
    walls,
    poles_ij: list[tuple[int, int]] | None,
    grid_cols: int,
    grid_rows: int,
    reject_interior_poles: bool = False,
) -> tuple[int, int, int, int]:
    """Expand cell AABB greedily while the block stays free of interior walls."""

    def _ok(nc0: int, nr0: int, nc1: int, nr1: int) -> bool:
        if nc0 < 0 or nr0 < 0 or nc1 >= grid_cols or nr1 >= grid_rows:
            return False
        if nc1 < nc0 or nr1 < nr0:
            return False
        return _block_is_cylinder_only(
            walls=walls,
            poles_ij=poles_ij,
            col0=nc0,
            row0=nr0,
            width=nc1 - nc0 + 1,
            height=nr1 - nr0 + 1,
            reject_interior_poles=reject_interior_poles,
        )

    # Seed must itself be open; if walls already cut through, still return it.
    if not _ok(c0, r0, c1, r1):
        return c0, r0, c1, r1

    changed = True
    while changed:
        changed = False
        if _ok(c0 - 1, r0, c1, r1):
            c0 -= 1
            changed = True
        if _ok(c0, r0, c1 + 1, r1):
            c1 += 1
            changed = True
        if _ok(c0, r0 - 1, c1, r1):
            r0 -= 1
            changed = True
        if _ok(c0, r0, c1, r1 + 1):
            r1 += 1
            changed = True
    return c0, r0, c1, r1


def _origins_covering_aabb(
    c0: int,
    r0: int,
    c1: int,
    r1: int,
    *,
    width: int,
    height: int,
    grid_cols: int,
    grid_rows: int,
) -> list[tuple[int, int]]:
    """All in-bounds top-lefts for a width×height block covering [c0..c1]×[r0..r1].

    Slack can sit on either side — callers pick among these (e.g. grow into open
    space above instead of into walls below).
    """
    need_w = c1 - c0 + 1
    need_h = r1 - r0 + 1
    if width < need_w or height < need_h:
        return []
    if width > grid_cols or height > grid_rows:
        return []
    # nc0 must satisfy: nc0 <= c0 and nc0 + width - 1 >= c1
    # ⇒ c1 - width + 1 <= nc0 <= c0, also 0 <= nc0 <= grid_cols - width
    col_lo = max(0, c1 - width + 1)
    col_hi = min(c0, grid_cols - width)
    row_lo = max(0, r1 - height + 1)
    row_hi = min(r0, grid_rows - height)
    if col_lo > col_hi or row_lo > row_hi:
        return []
    return [(nc0, nr0) for nr0 in range(row_lo, row_hi + 1) for nc0 in range(col_lo, col_hi + 1)]


def _expand_aabb_to_size(
    c0: int,
    r0: int,
    c1: int,
    r1: int,
    *,
    width: int,
    height: int,
    grid_cols: int,
    grid_rows: int,
) -> tuple[int, int] | None:
    """Return one covering top-left (centred slack); prefer ``_origins_covering_aabb``."""
    origins = _origins_covering_aabb(
        c0,
        r0,
        c1,
        r1,
        width=width,
        height=height,
        grid_cols=grid_cols,
        grid_rows=grid_rows,
    )
    if not origins:
        return None
    need_w = c1 - c0 + 1
    need_h = r1 - r0 + 1
    # Prefer nearly-centred placement among valid origins.
    target_c = c0 - (width - need_w) // 2
    target_r = r0 - (height - need_h) // 2
    return min(origins, key=lambda o: abs(o[0] - target_c) + abs(o[1] - target_r))


def _wall_is_interior(
    wall,
    *,
    col0: int,
    row0: int,
    width: int,
    height: int,
) -> bool:
    """True if the wall sits strictly inside the block (not on its rim)."""
    if wall.kind == "h":
        # Horizontal wall on pole-row ``a``; rim rows are row0 and row0+height.
        if not (row0 < wall.a < row0 + height):
            return False
        lo, hi = min(wall.b0, wall.b1), max(wall.b0, wall.b1)
        return lo < col0 + width and hi > col0
    # Vertical wall on pole-col ``a``; rim cols are col0 and col0+width.
    if not (col0 < wall.a < col0 + width):
        return False
    lo, hi = min(wall.b0, wall.b1), max(wall.b0, wall.b1)
    return lo < row0 + height and hi > row0


def _block_has_walls(
    walls,
    *,
    col0: int,
    row0: int,
    width: int,
    height: int,
) -> bool:
    if walls is None:
        return False
    return any(
        _wall_is_interior(w, col0=col0, row0=row0, width=width, height=height)
        for w in walls.walls
    )


def _block_has_interior_poles(
    poles_ij: list[tuple[int, int]] | None,
    *,
    col0: int,
    row0: int,
    width: int,
    height: int,
) -> bool:
    """True if a lattice pole sits strictly inside the block (not on its rim)."""
    if not poles_ij:
        return False
    for i, j in poles_ij:
        if col0 < int(i) < col0 + width and row0 < int(j) < row0 + height:
            return True
    return False


def _block_is_cylinder_only(
    *,
    walls,
    poles_ij: list[tuple[int, int]] | None,
    col0: int,
    row0: int,
    width: int,
    height: int,
    reject_interior_poles: bool = False,
) -> bool:
    """Open cylinder pad: no walls strictly inside.

    Lattice posts often remain in the pad, so interior poles are ignored unless
    ``reject_interior_poles`` is set.
    """
    if _block_has_walls(walls, col0=col0, row0=row0, width=width, height=height):
        return False
    if reject_interior_poles and _block_has_interior_poles(
        poles_ij, col0=col0, row0=row0, width=width, height=height
    ):
        return False
    return True


def cylinder_region_cells(
    cylinders: list[Cylinder],
    params: Params,
    *,
    walls=None,
    poles_ij: list[tuple[int, int]] | None = None,
) -> CylinderRegionCells | None:
    """Centres → grow open AABB until walls → snap to 5×5 else 5×4/4×5.

    1. Cells containing cylinder centres → seed cell AABB
    2. Grow that AABB while the block stays free of interior walls
    3. Prefer a 5×5 inside the grown pad that covers the centres; else 5×4 / 4×5
    4. If nothing configured fits, return the grown pad itself
    """
    maze = params.maze
    cfg = _cfg(params)
    cell_mm = float(maze["cell_size_mm"])
    grid_cols = int(maze["grid_cols"])
    grid_rows = int(maze["grid_rows"])
    reject_poles = bool(cfg.get("region_reject_interior_poles", False))

    seed = _cylinder_center_aabb(
        cylinders, cell_mm=cell_mm, grid_cols=grid_cols, grid_rows=grid_rows
    )
    if seed is None:
        return None
    sc0, sr0, sc1, sr1 = seed

    gc0, gr0, gc1, gr1 = _grow_aabb_until_walls(
        sc0,
        sr0,
        sc1,
        sr1,
        walls=walls,
        poles_ij=poles_ij,
        grid_cols=grid_cols,
        grid_rows=grid_rows,
        reject_interior_poles=reject_poles,
    )
    grown_w = gc1 - gc0 + 1
    grown_h = gr1 - gr0 + 1

    # Cover requirement: still must enclose the cylinder/hull seed.
    need_c0, need_r0, need_c1, need_r1 = sc0, sr0, sc1, sr1
    need_w = need_c1 - need_c0 + 1
    need_h = need_r1 - need_r0 + 1

    preferred = cfg.get("region_cell_sizes_preferred", [[5, 5]])
    fallback = cfg.get("region_cell_sizes_fallback", [[5, 4], [4, 5]])

    def _parse(sizes) -> list[tuple[int, int]]:
        out: list[tuple[int, int]] = []
        for t in sizes:
            if len(t) != 2:
                continue
            out.append((int(t[0]), int(t[1])))
        return out

    preferred_sizes = _parse(preferred) or [(5, 5)]
    fallback_sizes = _parse(fallback) or [(5, 4), (4, 5)]

    def _place(sizes: list[tuple[int, int]]) -> CylinderRegionCells | None:
        ordered = sorted(
            sizes,
            key=lambda s: (
                -(s[0] * s[1]),
                abs(s[0] - need_w) + abs(s[1] - need_h),
            ),
        )
        for tw, th in ordered:
            origins = _origins_covering_aabb(
                need_c0,
                need_r0,
                need_c1,
                need_r1,
                width=tw,
                height=th,
                grid_cols=grid_cols,
                grid_rows=grid_rows,
            )
            # Keep placement inside the grown open pad.
            origins = [
                (nc0, nr0)
                for nc0, nr0 in origins
                if nc0 >= gc0
                and nr0 >= gr0
                and nc0 + tw - 1 <= gc1
                and nr0 + th - 1 <= gr1
            ]
            t_c = need_c0 - (tw - need_w) // 2
            t_r = need_r0 - (th - need_h) // 2
            origins = sorted(
                origins, key=lambda o: abs(o[0] - t_c) + abs(o[1] - t_r)
            )
            for nc0, nr0 in origins:
                if not _block_is_cylinder_only(
                    walls=walls,
                    poles_ij=poles_ij,
                    col0=nc0,
                    row0=nr0,
                    width=tw,
                    height=th,
                    reject_interior_poles=reject_poles,
                ):
                    continue
                return CylinderRegionCells(
                    col0=nc0, row0=nr0, width=tw, height=th
                )
        return None

    hit = _place(preferred_sizes)
    if hit is not None:
        return hit
    hit = _place(fallback_sizes)
    if hit is not None:
        return hit
    # Grown pad itself (whatever open rectangle hull-expand reached).
    return CylinderRegionCells(
        col0=gc0, row0=gr0, width=grown_w, height=grown_h
    )


def cylinder_region_hull_mm(
    cylinders: list[Cylinder],
    *,
    margin_mm: float = 180.0,
    samples_per_disk: int = 32,
) -> np.ndarray | None:
    """Convex hull (N×2 mm) of cylinder disks grown by ``margin_mm``."""
    if not cylinders:
        return None
    n = max(8, int(samples_per_disk))
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    pts: list[list[float]] = []
    for c in cylinders:
        r = float(c.radius_mm) + float(margin_mm)
        if r <= 0:
            pts.append([float(c.x_mm), float(c.y_mm)])
            continue
        xs = float(c.x_mm) + r * np.cos(angles)
        ys = float(c.y_mm) + r * np.sin(angles)
        pts.extend(np.column_stack([xs, ys]).tolist())
    arr = np.asarray(pts, dtype=np.float64)
    if len(arr) < 3:
        return arr
    hull = cv2.convexHull(arr.astype(np.float32))
    return hull.reshape(-1, 2).astype(np.float64)


def paint_cylinder_hull_on(
    base_bgr: np.ndarray,
    fine: FineWarpResult,
    cylinders: list[Cylinder],
    params: Params,
    *,
    margin_mm: float | None = None,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Tint + outline the convex-hull cylinder region onto an existing image."""
    cfg = _cfg(params)
    if margin_mm is None:
        margin_mm = float(
            cfg.get(
                "region_margin_mm",
                float(params.maze.get("cell_size_mm", 180.0)),
            )
        )
    base = base_bgr.copy()
    hull_mm = cylinder_region_hull_mm(cylinders, margin_mm=margin_mm)
    if hull_mm is None or len(hull_mm) == 0:
        return base, None

    hull_px = np.array(
        [_mm_to_fine_px(float(x), float(y), fine, params) for x, y in hull_mm],
        dtype=np.int32,
    )

    tint = base.copy()
    cv2.fillConvexPoly(tint, hull_px, (0, 180, 255))
    overlay = cv2.addWeighted(tint, 0.35, base, 0.65, 0)
    cv2.polylines(overlay, [hull_px], True, (0, 220, 255), 3, cv2.LINE_AA)

    for c in cylinders:
        cx, cy = _mm_to_fine_px(float(c.x_mm), float(c.y_mm), fine, params)
        r_px = max(2, int(round(float(c.radius_mm) * float(fine.pixels_per_mm))))
        cv2.circle(overlay, (cx, cy), r_px, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(overlay, (cx, cy), 3, (0, 0, 255), -1, cv2.LINE_AA)

    top = hull_px[int(np.argmin(hull_px[:, 1]))]
    label = f"cylinder region (+{margin_mm:g} mm)"
    cv2.putText(
        overlay,
        label,
        (int(top[0]) + 8, max(24, int(top[1]) - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 220, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay, hull_mm


def paint_cylinder_region_on(
    base_bgr: np.ndarray,
    fine: FineWarpResult,
    cylinders: list[Cylinder],
    params: Params,
    *,
    walls=None,
    margin_mm: float | None = None,
) -> tuple[np.ndarray, CylinderRegionCells | None]:
    """Tint maze cells in the fitted cylinder block (all-walls overlay)."""
    del margin_mm
    base = base_bgr.copy()
    poles_ij = [(int(i), int(j)) for (i, j) in fine.poles_ij]
    region = cylinder_region_cells(
        cylinders, params, walls=walls, poles_ij=poles_ij
    )
    if region is None:
        return base, None

    cell_mm = float(params.maze["cell_size_mm"])
    tint = base.copy()
    for c, r in region.cells():
        p0 = _mm_to_fine_px(c * cell_mm, r * cell_mm, fine, params)
        p1 = _mm_to_fine_px((c + 1) * cell_mm, (r + 1) * cell_mm, fine, params)
        cv2.rectangle(tint, p0, p1, (0, 180, 255), thickness=-1)
    overlay = cv2.addWeighted(tint, 0.32, base, 0.68, 0)

    outer0 = _mm_to_fine_px(
        region.col0 * cell_mm, region.row0 * cell_mm, fine, params
    )
    outer1 = _mm_to_fine_px(
        (region.col1 + 1) * cell_mm, (region.row1 + 1) * cell_mm, fine, params
    )
    cv2.rectangle(overlay, outer0, outer1, (0, 220, 255), 3, cv2.LINE_AA)

    for c, r in region.cells():
        p0 = _mm_to_fine_px(c * cell_mm, r * cell_mm, fine, params)
        p1 = _mm_to_fine_px((c + 1) * cell_mm, (r + 1) * cell_mm, fine, params)
        cv2.rectangle(overlay, p0, p1, (0, 200, 255), 1, cv2.LINE_AA)
        cx = (p0[0] + p1[0]) // 2
        cy = (p0[1] + p1[1]) // 2
        label = f"{c},{r}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.putText(
            overlay,
            label,
            (cx - tw // 2, cy + th // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )

    for cyl in cylinders:
        cx, cy = _mm_to_fine_px(float(cyl.x_mm), float(cyl.y_mm), fine, params)
        r_px = max(2, int(round(float(cyl.radius_mm) * float(fine.pixels_per_mm))))
        cv2.circle(overlay, (cx, cy), r_px, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(overlay, (cx, cy), 3, (0, 0, 255), -1, cv2.LINE_AA)

    title = f"cylinder cells {region.width}x{region.height} @ ({region.col0},{region.row0})"
    cv2.putText(
        overlay,
        title,
        (outer0[0] + 8, max(24, outer0[1] - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 220, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay, region


def draw_cylinder_hull(
    fine: FineWarpResult,
    cylinders: list[Cylinder],
    params: Params,
    *,
    margin_mm: float | None = None,
) -> tuple[np.ndarray, np.ndarray | None]:
    """Convex-hull cylinder region on the fine-warp image (05b_region)."""
    return paint_cylinder_hull_on(
        fine.warped_bgr, fine, cylinders, params, margin_mm=margin_mm
    )


def save_cylinders_debug(
    detection: CylinderDetection,
    output_dir: str | Path,
    *,
    prefix: str = "05b_cylinders",
    fine: FineWarpResult | None = None,
    params: Params | None = None,
) -> CylinderRegionCells | None:
    """Write cylinder debug artefacts. ``_region.png`` is the convex hull."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_mask.png"), detection.mask)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), detection.overlay_bgr)
    cv2.imwrite(str(out / f"{prefix}_overlay_warp.png"), detection.overlay_warp_bgr)

    hull_mm: np.ndarray | None = None
    margin_mm: float | None = None
    if fine is not None and params is not None and detection.cylinders:
        margin_mm = float(
            _cfg(params).get(
                "region_margin_mm",
                float(params.maze.get("cell_size_mm", 180.0)),
            )
        )
        region_bgr, hull_mm = draw_cylinder_hull(
            fine, detection.cylinders, params, margin_mm=margin_mm
        )
        cv2.imwrite(str(out / f"{prefix}_region.png"), region_bgr)
        # Cell block needs walls; written later in save_combined_walls.

    rows = np.array(
        [
            [
                c.x_mm,
                c.y_mm,
                c.radius_mm,
                c.src_xy[0],
                c.src_xy[1],
                c.warp_xy[0],
                c.warp_xy[1],
                c.radius_warp_px,
                c.circularity,
                c.fill,
            ]
            for c in detection.cylinders
        ],
        dtype=np.float64,
    )
    if len(rows) == 0:
        rows = np.zeros((0, 10), dtype=np.float64)
    np.savetxt(
        out / f"{prefix}_xy.txt",
        rows,
        fmt="%.3f",
        header=(
            "x_mm y_mm radius_mm src_x src_y warp_x warp_y "
            "radius_warp_px circularity fill"
        ),
        comments="",
    )
    payload: dict = {
        "count": len(detection.cylinders),
        "cylinders": [
            {
                "x_mm": c.x_mm,
                "y_mm": c.y_mm,
                "radius_mm": c.radius_mm,
                "src_xy": [c.src_xy[0], c.src_xy[1]],
                "warp_xy": [c.warp_xy[0], c.warp_xy[1]],
                "radius_warp_px": c.radius_warp_px,
                "circularity": c.circularity,
                "fill": c.fill,
            }
            for c in detection.cylinders
        ],
    }
    if hull_mm is not None:
        payload["region_hull"] = {
            "kind": "convex_hull",
            "margin_mm": margin_mm,
            "hull_mm": [[float(x), float(y)] for x, y in hull_mm],
        }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    return None


def load_cylinders_from_yaml(path: str | Path) -> list[Cylinder]:
    """Read cylinder list from ``05b_cylinders.yaml``."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    out: list[Cylinder] = []
    for raw in data.get("cylinders") or []:
        src = raw.get("src_xy") or [0.0, 0.0]
        warp = raw.get("warp_xy") or [0.0, 0.0]
        out.append(
            Cylinder(
                x_mm=float(raw["x_mm"]),
                y_mm=float(raw["y_mm"]),
                radius_mm=float(raw["radius_mm"]),
                src_xy=(float(src[0]), float(src[1])),
                warp_xy=(float(warp[0]), float(warp[1])),
                radius_warp_px=float(raw.get("radius_warp_px") or 0.0),
                circularity=float(raw.get("circularity") or 0.0),
                fill=float(raw.get("fill") or 0.0),
            )
        )
    return out


def append_region_cells_yaml(
    output_dir: str | Path,
    region: CylinderRegionCells,
    *,
    cylinders_prefix: str = "05b_cylinders",
) -> None:
    """Merge fitted cell-block into the cylinders yaml (after walls exist)."""
    path = Path(output_dir) / f"{cylinders_prefix}.yaml"
    payload: dict = {}
    if path.is_file():
        with open(path, encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
            if isinstance(loaded, dict):
                payload = loaded
    payload["region_cells"] = {
        "kind": "cell_block",
        "col0": region.col0,
        "row0": region.row0,
        "width": region.width,
        "height": region.height,
        "cells": [[c, r] for c, r in region.cells()],
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)


def filter_near_poles(
    cylinders: list[Cylinder],
    poles_mm: list[tuple[float, float]],
    *,
    min_dist_mm: float,
) -> list[Cylinder]:
    """Optional helper: drop centres sitting on a post (false round blobs)."""
    if not cylinders or not poles_mm or min_dist_mm <= 0:
        return cylinders
    tree = cKDTree(np.asarray(poles_mm, dtype=np.float64))
    kept: list[Cylinder] = []
    for c in cylinders:
        d, _ = tree.query([c.x_mm, c.y_mm], k=1)
        if float(d) >= min_dist_mm:
            kept.append(c)
    return kept
