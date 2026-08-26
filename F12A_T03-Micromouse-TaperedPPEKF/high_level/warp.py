from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

from poles import Params, chamfer_missing_poles


CORNER_ORDER = ("TL", "TR", "BR", "BL")


@dataclass
class WarpResult:
    warped_bgr: np.ndarray
    matrix: np.ndarray
    ordered_corners_px: np.ndarray
    output_size_px: int


@dataclass
class FineWarpResult:
    warped_bgr: np.ndarray
    matrix: np.ndarray
    poles_ij: np.ndarray
    poles_src_px: np.ndarray
    poles_dst_px: np.ndarray
    px_per_cell: float
    pixels_per_mm: float
    canvas_size_px: tuple[int, int]


def _orient_outer_corners(
    outer_corners_px: np.ndarray,
    colour_centroids_px: dict[str, np.ndarray],
    orientation: dict[str, str],
) -> np.ndarray:
    if outer_corners_px.shape != (4, 2):
        raise ValueError("outer_corners_px must be (4, 2)")

    role_to_colour = {role.upper(): name for name, role in orientation.items()}
    missing = [r for r in CORNER_ORDER if r not in role_to_colour]
    if missing:
        raise ValueError(f"warp.orientation missing roles: {missing}")

    used: set[int] = set()
    ordered = np.zeros((4, 2), dtype=np.float32)
    for i, role in enumerate(CORNER_ORDER):
        colour = role_to_colour[role]
        if colour not in colour_centroids_px:
            raise RuntimeError(f"missing colour centroid for '{colour}' ({role})")
        centroid = colour_centroids_px[colour]
        dists = np.linalg.norm(outer_corners_px - centroid, axis=1)
        for idx in np.argsort(dists):
            j = int(idx)
            if j not in used:
                used.add(j)
                ordered[i] = outer_corners_px[j]
                break
        else:
            raise RuntimeError(f"could not assign outer corner for {role}")
    return ordered


def warp_from_outer_corners(
    image_bgr: np.ndarray,
    outer_corners_px: np.ndarray,
    colour_centroids_px: dict[str, np.ndarray],
    params: Params,
) -> WarpResult:
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("image_bgr is empty")

    cfg = params.warp
    size = int(cfg["output_size_px"])
    ordered = _orient_outer_corners(
        outer_corners_px,
        colour_centroids_px,
        cfg["orientation"],
    )

    src = ordered.astype(np.float32)
    dst = np.array(
        [
            [0, 0],
            [size - 1, 0],
            [size - 1, size - 1],
            [0, size - 1],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(
        image_bgr,
        matrix,
        (size, size),
        flags=cv2.INTER_LINEAR,
    )

    return WarpResult(
        warped_bgr=warped,
        matrix=matrix,
        ordered_corners_px=ordered,
        output_size_px=size,
    )


def project_points(points_px: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if len(points_px) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    pts = points_px.reshape(-1, 1, 2).astype(np.float32)
    out = cv2.perspectiveTransform(pts, matrix)
    return out.reshape(-1, 2)


def _trim_axis_outliers(
    poles_xy: np.ndarray,
    spacing: float,
    *,
    gap_frac: float = 0.55,
    pack_gap_frac: float = 1.2,
    max_pack: int = 3,
) -> np.ndarray:
    """Mask that drops poles isolated past the lattice on an axis.

    A single warped outlier (clamped to the image edge, far outside the frame)
    otherwise inflates min/max span or shifts the rounded origin so a whole
    lattice row maps OOB / collapses. Also drops a small packed group of
    outliers separated from the main body by more than ~one cell (maze12
    bottom). Pack gaps must exceed pack_gap_frac so sparse chamfer edge
    columns (≤3 poles) are not mistaken for outliers.
    """
    keep = np.ones(len(poles_xy), dtype=bool)
    if len(poles_xy) < 4 or spacing <= 1e-3:
        return keep
    gap_single = gap_frac * spacing
    gap_pack = pack_gap_frac * spacing
    for ax in (0, 1):
        order = np.argsort(poles_xy[:, ax])
        alive = order[keep[order]]
        if len(alive) >= 4 and float(poles_xy[alive[1], ax] - poles_xy[alive[0], ax]) > gap_single:
            keep[alive[0]] = False
        alive = order[keep[order]]
        if len(alive) >= 4 and float(poles_xy[alive[-1], ax] - poles_xy[alive[-2], ax]) > gap_single:
            keep[alive[-1]] = False

        # Packed outliers beyond the board (gap >> one lattice step).
        changed = True
        while changed:
            changed = False
            alive = order[keep[order]]
            if len(alive) < 4:
                break
            vals = poles_xy[alive, ax]
            gaps = np.diff(vals)
            for bi, g in enumerate(gaps):
                if g <= gap_pack:
                    continue
                left_n = bi + 1
                right_n = len(alive) - left_n
                if left_n <= max_pack:
                    keep[alive[:left_n]] = False
                    changed = True
                    break
                if right_n <= max_pack:
                    keep[alive[left_n:]] = False
                    changed = True
                    break
    return keep


def _count_lattice_oob(
    pw: np.ndarray,
    spacing: float,
    pole_grid_cols: int,
    pole_grid_rows: int,
) -> int:
    origin = pw.min(axis=0)
    ij = np.round((pw - origin) / spacing).astype(np.int32)
    origin = np.array(
        [
            float(np.median(pw[:, 0] - ij[:, 0] * spacing)),
            float(np.median(pw[:, 1] - ij[:, 1] * spacing)),
        ],
        dtype=np.float64,
    )
    ij = np.round((pw - origin) / spacing).astype(np.int32)
    return int(
        (
            (ij[:, 0] < 0)
            | (ij[:, 0] >= pole_grid_cols)
            | (ij[:, 1] < 0)
            | (ij[:, 1] >= pole_grid_rows)
        ).sum()
    )


def assign_pole_lattice(
    poles_warped_px: np.ndarray,
    pole_grid_cols: int,
    pole_grid_rows: int,
    *,
    skip_ij: set[tuple[int, int]] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (ij, keep_indices) for poles that land on unique in-range lattice sites.

    `skip_ij` drops chamfer corner slots (and anything else) so false detections
    outside the board cannot claim those empty grid positions.
    """
    if len(poles_warped_px) < 4:
        raise ValueError("need at least 4 poles for lattice assignment")
    skip = skip_ij or set()

    tree = cKDTree(poles_warped_px)
    nn = np.array(
        [tree.query(p, k=2)[0][1] for p in poles_warped_px],
        dtype=np.float64,
    )
    spacing = float(np.median(nn))
    if spacing <= 1e-3:
        raise RuntimeError("pole spacing too small for lattice assignment")

    center = np.median(poles_warped_px, axis=0)
    # 10x10 span is ~9 cells; corner is ~6.4 spacings from centre — keep a bit more.
    max_radius = 7.0 * spacing
    cluster = np.linalg.norm(poles_warped_px - center, axis=1) <= max_radius
    if int(cluster.sum()) < 4:
        raise RuntimeError("pole cluster too small for lattice assignment")

    cluster_idx = np.flatnonzero(cluster)
    pw = poles_warped_px[cluster]
    trim = _trim_axis_outliers(pw, spacing)
    if int(trim.sum()) >= 4:
        pw = pw[trim]
        cluster_idx = cluster_idx[trim]

    # Median NN can run ~5–10% low when close pairs exist, pushing edge poles
    # OOB. Only expand spacing from the observed span when that actually
    # happens — raw span/NN ratio alone is fooled by residual outliers (maze12).
    span_x = float(pw[:, 0].max() - pw[:, 0].min())
    span_y = float(pw[:, 1].max() - pw[:, 1].min())
    expect_i = float(max(1, pole_grid_cols - 1))
    expect_j = float(max(1, pole_grid_rows - 1))
    if _count_lattice_oob(pw, spacing, pole_grid_cols, pole_grid_rows) >= 4:
        spacing = max(spacing, span_x / expect_i, span_y / expect_j)

    origin = pw.min(axis=0)
    ij_local = np.round((pw - origin) / spacing).astype(np.int32)

    origin = np.array(
        [
            float(np.median(pw[:, 0] - ij_local[:, 0] * spacing)),
            float(np.median(pw[:, 1] - ij_local[:, 1] * spacing)),
        ],
        dtype=np.float64,
    )
    ij_local = np.round((pw - origin) / spacing).astype(np.int32)

    best: dict[tuple[int, int], tuple[float, int]] = {}
    for local_k, (i, j) in enumerate(ij_local):
        if not (0 <= i < pole_grid_cols and 0 <= j < pole_grid_rows):
            continue
        key = (int(i), int(j))
        if key in skip:
            continue
        ideal = origin + np.array([i, j], dtype=np.float64) * spacing
        err = float(np.linalg.norm(pw[local_k] - ideal))
        global_k = int(cluster_idx[local_k])
        if key not in best or err < best[key][0]:
            best[key] = (err, global_k)

    if len(best) < 4:
        raise RuntimeError("too few poles after lattice filtering")

    ordered = sorted(best.items(), key=lambda kv: kv[1][1])
    ij_kept = np.array([list(k) for k, _ in ordered], dtype=np.int32)
    keep = np.array([v[1] for _, v in ordered], dtype=np.int32)
    return ij_kept, keep


def fine_warp_from_poles(
    image_bgr: np.ndarray,
    poles_src_px: np.ndarray,
    coarse: WarpResult,
    params: Params,
) -> FineWarpResult:
    maze = params.maze
    fine_cfg = params.warp["fine"]
    poles_cfg = params.poles
    pole_cols = int(maze["pole_grid_cols"])
    pole_rows = int(maze["pole_grid_rows"])
    cell_mm = float(maze["cell_size_mm"])
    px_per_cell = float(fine_cfg["px_per_cell"])
    margin_cells = float(fine_cfg["margin_cells"])

    skip = (
        chamfer_missing_poles(pole_cols, pole_rows)
        if bool(poles_cfg.get("skip_chamfer_poles", True))
        else set()
    )

    poles_warped = project_points(poles_src_px, coarse.matrix)
    # Coarse warp clamps out-of-frame projections onto the image edge. Those
    # border poles inflate lattice span / origin and shift the fine warp
    # (maze12 left/right collapse). Drop them before assignment.
    h_c, w_c = coarse.warped_bgr.shape[:2]
    border_m = 6.0
    inside = (
        (poles_warped[:, 0] >= border_m)
        & (poles_warped[:, 0] <= (w_c - 1.0 - border_m))
        & (poles_warped[:, 1] >= border_m)
        & (poles_warped[:, 1] <= (h_c - 1.0 - border_m))
    )
    if int(inside.sum()) >= 4:
        poles_src_px = poles_src_px[inside]
        poles_warped = poles_warped[inside]

    ij, keep = assign_pole_lattice(
        poles_warped, pole_cols, pole_rows, skip_ij=skip
    )
    src = poles_src_px[keep].astype(np.float32)

    dst = np.column_stack(
        [
            (ij[:, 0] + margin_cells) * px_per_cell,
            (ij[:, 1] + margin_cells) * px_per_cell,
        ]
    ).astype(np.float32)

    matrix, inliers = cv2.findHomography(
        src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0
    )
    if matrix is None:
        raise RuntimeError("findHomography failed")

    width = int(round((pole_cols - 1 + 2 * margin_cells) * px_per_cell))
    height = int(round((pole_rows - 1 + 2 * margin_cells) * px_per_cell))
    warped = cv2.warpPerspective(
        image_bgr,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
    )

    n_inliers = int(inliers.sum()) if inliers is not None else len(src)
    if n_inliers < 4:
        raise RuntimeError(f"too few homography inliers: {n_inliers}")

    return FineWarpResult(
        warped_bgr=warped,
        matrix=matrix,
        poles_ij=ij,
        poles_src_px=src,
        poles_dst_px=dst,
        px_per_cell=px_per_cell,
        pixels_per_mm=px_per_cell / cell_mm,
        canvas_size_px=(width, height),
    )


def save_warp_debug(
    result: WarpResult,
    output_dir: str | Path,
    *,
    prefix: str = "03_warp",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}.png"), result.warped_bgr)
    np.savetxt(
        out / f"{prefix}_src_corners_xy.txt",
        result.ordered_corners_px,
        fmt="%.3f",
        header="x_px y_px  (TL TR BR BL in source image)",
        comments="",
    )
    np.save(out / f"{prefix}_matrix.npy", result.matrix)


def recover_poles_on_lattice(
    fine: FineWarpResult,
    params: Params,
    *,
    min_cyan_px: float | None = None,
    search_radius_px: float | None = None,
) -> tuple[FineWarpResult, int]:
    """Fill empty lattice sites that still show cyan in the fine-warp view.

    Source HSV detection / lattice assignment sometimes drops edge poles
    (maze13 top row, cylinder5 left rail). After the fine warp those posts
    sit on known ideal `(i,j)` coordinates, so a local cyan search recovers
    them without re-running the homography.
    """
    maze = params.maze
    poles_cfg = params.poles
    pole_cols = int(maze["pole_grid_cols"])
    pole_rows = int(maze["pole_grid_rows"])
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)

    skip = (
        chamfer_missing_poles(pole_cols, pole_rows)
        if bool(poles_cfg.get("skip_chamfer_poles", True))
        else set()
    )
    present = {(int(i), int(j)) for i, j in fine.poles_ij}

    warped = fine.warped_bgr
    hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(poles_cfg["hsv_lo"], dtype=np.uint8),
        np.array(poles_cfg["hsv_hi"], dtype=np.uint8),
    )
    k = max(3, int(poles_cfg.get("morph_kernel", 3)))
    if k % 2 == 0:
        k += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # ~post footprint at px_per_cell; 40 cyan px ≈ a solid tip in the window.
    r_search = float(
        search_radius_px
        if search_radius_px is not None
        else poles_cfg.get("lattice_recover_radius_px", max(12.0, 0.22 * ppc))
    )
    min_cyan = float(
        min_cyan_px
        if min_cyan_px is not None
        else poles_cfg.get("lattice_recover_min_cyan_px", 40.0)
    )

    inv = np.linalg.inv(fine.matrix.astype(np.float64)).astype(np.float32)
    h, w = mask.shape
    new_ij: list[tuple[int, int]] = []
    new_src: list[np.ndarray] = []
    new_dst: list[np.ndarray] = []

    for i in range(pole_cols):
        for j in range(pole_rows):
            if (i, j) in skip or (i, j) in present:
                continue
            cx = (i + margin) * ppc
            cy = (j + margin) * ppc
            xi, yi = int(round(cx)), int(round(cy))
            rad = int(round(r_search))
            x0, x1 = max(0, xi - rad), min(w, xi + rad + 1)
            y0, y1 = max(0, yi - rad), min(h, yi + rad + 1)
            patch = mask[y0:y1, x0:x1]
            cyan = int(patch.sum() // 255)
            if cyan < min_cyan:
                continue
            ys, xs = np.where(patch > 0)
            if len(xs) < min_cyan:
                continue
            wx = float(xs.mean()) + x0
            wy = float(ys.mean()) + y0
            src = project_points(
                np.array([[wx, wy]], dtype=np.float32), inv
            )[0]
            new_ij.append((i, j))
            new_src.append(src.astype(np.float32))
            new_dst.append(
                np.array([(i + margin) * ppc, (j + margin) * ppc], dtype=np.float32)
            )

    if not new_ij:
        return fine, 0

    ij = np.vstack(
        [fine.poles_ij, np.asarray(new_ij, dtype=np.int32)]
    )
    src = np.vstack(
        [fine.poles_src_px, np.asarray(new_src, dtype=np.float32)]
    )
    dst = np.vstack(
        [fine.poles_dst_px, np.asarray(new_dst, dtype=np.float32)]
    )
    order = np.lexsort((ij[:, 0], ij[:, 1]))
    recovered = FineWarpResult(
        warped_bgr=fine.warped_bgr,
        matrix=fine.matrix,
        poles_ij=ij[order],
        poles_src_px=src[order],
        poles_dst_px=dst[order],
        px_per_cell=fine.px_per_cell,
        pixels_per_mm=fine.pixels_per_mm,
        canvas_size_px=fine.canvas_size_px,
    )
    return recovered, len(new_ij)


def _cyan_mask_warp(fine: FineWarpResult, params: Params) -> np.ndarray:
    poles_cfg = params.poles
    hsv = cv2.cvtColor(fine.warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(poles_cfg["hsv_lo"], dtype=np.uint8),
        np.array(poles_cfg["hsv_hi"], dtype=np.uint8),
    )
    k = max(3, int(poles_cfg.get("morph_kernel", 3)))
    if k % 2 == 0:
        k += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def filter_poles_lacking_cyan(
    fine: FineWarpResult,
    params: Params,
    *,
    min_cyan_px: float | None = None,
    search_radius_px: float | None = None,
) -> tuple[FineWarpResult, int]:
    """Drop lattice poles whose fine-warp neighbourhood has no cyan.

    Mis-indexed detections (e.g. dark cylinder rim → lattice hole with white
    floor) leave yellow dots / walls with no real post. Mirror the recover
    threshold so keep/drop is consistent.
    """
    if len(fine.poles_ij) == 0:
        return fine, 0

    poles_cfg = params.poles
    ppc = float(fine.px_per_cell)
    r_search = float(
        search_radius_px
        if search_radius_px is not None
        else poles_cfg.get("lattice_recover_radius_px", max(12.0, 0.22 * ppc))
    )
    min_cyan = float(
        min_cyan_px
        if min_cyan_px is not None
        else poles_cfg.get("lattice_recover_min_cyan_px", 40.0)
    )
    mask = _cyan_mask_warp(fine, params)
    h, w = mask.shape
    rad = int(round(r_search))

    keep: list[int] = []
    for k, (i, j) in enumerate(fine.poles_ij):
        # Prefer ideal lattice site; fall back to measured dst.
        margin = float(params.warp["fine"]["margin_cells"])
        cx = (int(i) + margin) * ppc
        cy = (int(j) + margin) * ppc
        xi, yi = int(round(cx)), int(round(cy))
        x0, x1 = max(0, xi - rad), min(w, xi + rad + 1)
        y0, y1 = max(0, yi - rad), min(h, yi + rad + 1)
        cyan = int(mask[y0:y1, x0:x1].sum() // 255)
        if cyan >= min_cyan:
            keep.append(k)

    dropped = len(fine.poles_ij) - len(keep)
    if dropped == 0:
        return fine, 0
    if len(keep) < 4:
        # Don't collapse the lattice if thresholds are too harsh for a frame.
        return fine, 0

    idx = np.asarray(keep, dtype=np.int32)
    return (
        FineWarpResult(
            warped_bgr=fine.warped_bgr,
            matrix=fine.matrix,
            poles_ij=fine.poles_ij[idx],
            poles_src_px=fine.poles_src_px[idx],
            poles_dst_px=fine.poles_dst_px[idx],
            px_per_cell=fine.px_per_cell,
            pixels_per_mm=fine.pixels_per_mm,
            canvas_size_px=fine.canvas_size_px,
        ),
        dropped,
    )


def load_fine_warp(
    output_dir: str | Path,
    params: Params,
    *,
    prefix: str = "04_fine_warp",
) -> FineWarpResult:
    """Reload ``04_fine_warp`` artifacts written by ``save_fine_warp_debug``."""
    out = Path(output_dir)
    png = out / f"{prefix}.png"
    mat_path = out / f"{prefix}_matrix.npy"
    poles_path = out / f"{prefix}_poles_ij.txt"
    warped = cv2.imread(str(png))
    if warped is None:
        raise FileNotFoundError(f"missing fine warp image: {png}")
    if not mat_path.is_file() or not poles_path.is_file():
        raise FileNotFoundError(
            f"missing fine warp lattice ({mat_path.name} / {poles_path.name})"
        )
    matrix = np.load(mat_path)
    data = np.loadtxt(poles_path, skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.size == 0:
        raise RuntimeError(f"no poles in {poles_path}")
    poles_ij = np.asarray(np.rint(data[:, 0:2]), dtype=np.int32)
    poles_src = np.asarray(data[:, 2:4], dtype=np.float32)
    poles_dst = np.asarray(data[:, 4:6], dtype=np.float32)
    h, w = warped.shape[:2]
    px_per_cell = float(params.warp["fine"]["px_per_cell"])
    cell_mm = float(params.maze["cell_size_mm"])
    return FineWarpResult(
        warped_bgr=warped,
        matrix=np.asarray(matrix, dtype=np.float64),
        poles_ij=poles_ij,
        poles_src_px=poles_src,
        poles_dst_px=poles_dst,
        px_per_cell=px_per_cell,
        pixels_per_mm=px_per_cell / cell_mm,
        canvas_size_px=(int(w), int(h)),
    )


def save_fine_warp_debug(
    result: FineWarpResult,
    output_dir: str | Path,
    *,
    prefix: str = "04_fine_warp",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}.png"), result.warped_bgr)
    np.savetxt(
        out / f"{prefix}_poles_ij.txt",
        np.column_stack([result.poles_ij, result.poles_src_px, result.poles_dst_px]),
        fmt="%.3f",
        header="i j src_x src_y dst_x dst_y",
        comments="",
    )
    np.save(out / f"{prefix}_matrix.npy", result.matrix)
