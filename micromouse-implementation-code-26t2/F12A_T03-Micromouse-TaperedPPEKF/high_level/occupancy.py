from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy import ndimage

from poles import Params
from shell import ShellResult
from walls import WallDetection, WallSeg, _pole_map
from warp import FineWarpResult


@dataclass
class OccupancyMap:
    occupancy: np.ndarray  # uint8, 1 = occupied
    inflated: np.ndarray
    cell_mm: float
    origin_mm: tuple[float, float]  # world mm of grid cell (0,0) centre-ish / corner
    width_mm: float
    height_mm: float
    overlay_bgr: np.ndarray
    preview_bgr: np.ndarray


def _mm_to_cell(x_mm: float, y_mm: float, origin: tuple[float, float], cell_mm: float) -> tuple[int, int]:
    return (
        int(round((x_mm - origin[0]) / cell_mm)),
        int(round((y_mm - origin[1]) / cell_mm)),
    )


def _inflate(mask: np.ndarray, radius_cells: int, shape: str = "diamond") -> np.ndarray:
    """Grow occupied cells by robot clearance.

    shape:
      disk    — Euclidean circle (good for any yaw; use circumradius)
      square  — axis-aligned square (Chebyshev); half-side of body
      diamond — Manhattan / sharp 4-connected; half-side, not a square body
    """
    if radius_cells <= 0:
        return mask.copy()
    mode = shape.lower().strip()
    free = mask == 0
    if mode in ("disk", "circle", "curved", "euclidean"):
        dist = ndimage.distance_transform_edt(free)
        return ((mask > 0) | (dist <= float(radius_cells))).astype(np.uint8)
    if mode in ("square", "box", "chebyshev"):
        dist = ndimage.distance_transform_cdt(free, metric="chessboard")
        return ((mask > 0) | (dist <= radius_cells)).astype(np.uint8)
    # diamond / sharp: iterated 4-neighbour dilation (Manhattan)
    struct = ndimage.generate_binary_structure(2, 1)
    return ndimage.binary_dilation(
        mask > 0, structure=struct, iterations=radius_cells
    ).astype(np.uint8)


def _draw_shell_band(
    mask: np.ndarray,
    inner_mm: np.ndarray,
    outer_mm: np.ndarray,
    origin: tuple[float, float],
    cell_mm: float,
) -> None:
    pts = np.vstack([inner_mm, outer_mm[::-1]])
    cells = np.array(
        [_mm_to_cell(float(x), float(y), origin, cell_mm) for x, y in pts],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(mask, cells, 1)


def _draw_inner_wall(
    mask: np.ndarray,
    wall: WallSeg,
    poles_mm: dict[tuple[int, int], tuple[float, float]],
    origin: tuple[float, float],
    cell_mm: float,
    thickness_mm: float,
) -> None:
    if wall.kind == "h":
        a = poles_mm.get((wall.b0, wall.a))
        b = poles_mm.get((wall.b1, wall.a))
    else:
        a = poles_mm.get((wall.a, wall.b0))
        b = poles_mm.get((wall.a, wall.b1))
    if a is None or b is None:
        return
    p0 = _mm_to_cell(a[0], a[1], origin, cell_mm)
    p1 = _mm_to_cell(b[0], b[1], origin, cell_mm)
    thick = max(1, int(round(thickness_mm / cell_mm)))
    cv2.line(mask, p0, p1, 1, thickness=thick, lineType=cv2.LINE_8)


def _draw_post(
    mask: np.ndarray,
    x_mm: float,
    y_mm: float,
    post_mm: float,
    origin: tuple[float, float],
    cell_mm: float,
) -> None:
    half = post_mm / 2.0
    x0, y0 = _mm_to_cell(x_mm - half, y_mm - half, origin, cell_mm)
    x1, y1 = _mm_to_cell(x_mm + half, y_mm + half, origin, cell_mm)
    cv2.rectangle(mask, (x0, y0), (x1, y1), 1, thickness=-1)


def _draw_cylinder(
    mask: np.ndarray,
    x_mm: float,
    y_mm: float,
    radius_mm: float,
    origin: tuple[float, float],
    cell_mm: float,
) -> None:
    cx, cy = _mm_to_cell(x_mm, y_mm, origin, cell_mm)
    r = max(1, int(round(radius_mm / cell_mm)))
    cv2.circle(mask, (cx, cy), r, 1, thickness=-1, lineType=cv2.LINE_8)


def _maze_cell_block_mask(
    *,
    col0: int,
    row0: int,
    col1: int,
    row1: int,
    maze_cell_mm: float,
    origin: tuple[float, float],
    cell_mm: float,
    shape_hw: tuple[int, int],
) -> np.ndarray:
    """Occupancy-grid mask covering maze cells [col0..col1] × [row0..row1]."""
    mask = np.zeros(shape_hw, dtype=np.uint8)
    x0 = float(col0) * maze_cell_mm
    y0 = float(row0) * maze_cell_mm
    x1 = float(col1 + 1) * maze_cell_mm
    y1 = float(row1 + 1) * maze_cell_mm
    p0 = _mm_to_cell(x0, y0, origin, cell_mm)
    p1 = _mm_to_cell(x1, y1, origin, cell_mm)
    x_lo, x_hi = sorted((p0[0], p1[0]))
    y_lo, y_hi = sorted((p0[1], p1[1]))
    cv2.rectangle(mask, (x_lo, y_lo), (x_hi, y_hi), 1, thickness=-1)
    return mask


def build_occupancy(
    fine: FineWarpResult,
    walls: WallDetection,
    shell: ShellResult,
    params: Params,
    cylinders: list | None = None,
) -> OccupancyMap:
    maze = params.maze
    map_cfg = params.map
    robot = params.robot

    cell_mm = float(map_cfg["occupancy_cell_mm"])
    margin_mm = float(map_cfg.get("margin_mm", 20.0))
    inflate_shape = str(map_cfg.get("inflate_shape", "diamond"))
    wall_pad = float(map_cfg.get("wall_padding_mm", 0.0))
    post_pad = float(map_cfg.get("post_padding_mm", 0.0))
    shell_pad = float(map_cfg.get("shell_padding_mm", 0.0))
    cyl_pad = float(map_cfg.get("cylinder_padding_mm", 0.0))
    wall_th = float(maze["wall_thickness_mm"]) + 2.0 * wall_pad
    post_mm = float(maze["post_size_mm"]) + 2.0 * post_pad
    maze_cell = float(maze["cell_size_mm"])
    cylinders = cylinders or []

    left, top, right, bottom = [float(v) for v in shell.meta["outer_box_mm"]]
    origin = (left - margin_mm, top - margin_mm)
    width_mm = (right - left) + 2.0 * margin_mm
    height_mm = (bottom - top) + 2.0 * margin_mm
    cols = int(np.ceil(width_mm / cell_mm))
    rows = int(np.ceil(height_mm / cell_mm))

    shell_mask = np.zeros((rows, cols), dtype=np.uint8)
    wall_mask = np.zeros((rows, cols), dtype=np.uint8)
    post_mask = np.zeros((rows, cols), dtype=np.uint8)
    cyl_mask = np.zeros((rows, cols), dtype=np.uint8)

    for seg in shell.segments:
        _draw_shell_band(shell_mask, seg.inner_mm, seg.outer_mm, origin, cell_mm)
    if shell_pad > 0:
        shell_mask = _inflate(
            shell_mask, int(np.ceil(shell_pad / cell_mm)), inflate_shape
        )

    poles_mm = {
        (i, j): (float(i) * maze_cell, float(j) * maze_cell)
        for (i, j) in _pole_map(fine, params)
    }
    for wall in walls.walls:
        _draw_inner_wall(wall_mask, wall, poles_mm, origin, cell_mm, wall_th)
    for (_i, _j), (x, y) in poles_mm.items():
        _draw_post(post_mask, x, y, post_mm, origin, cell_mm)

    # Physical cylinder bodies always. Extra cylinder_padding_mm is clipped to
    # the cylinder cell block so large clearance cannot bleed into neighbour
    # maze corridors outside the open pad. Robot inflate for cylinders is
    # applied separately and also clipped to that block.
    clip_pad = bool(map_cfg.get("clip_cylinder_padding_to_region", True))
    region = None
    region_mask: np.ndarray | None = None
    if cylinders and clip_pad:
        from cylinders import cylinder_region_cells

        poles_ij = [(int(i), int(j)) for (i, j) in fine.poles_ij]
        region = cylinder_region_cells(
            cylinders, params, walls=walls, poles_ij=poles_ij
        )
        if region is not None:
            region_mask = _maze_cell_block_mask(
                col0=region.col0,
                row0=region.row0,
                col1=region.col1,
                row1=region.row1,
                maze_cell_mm=maze_cell,
                origin=origin,
                cell_mm=cell_mm,
                shape_hw=(rows, cols),
            )

    pad_mask = np.zeros((rows, cols), dtype=np.uint8)
    for cyl in cylinders:
        r_phys = float(cyl.radius_mm)
        _draw_cylinder(
            cyl_mask, float(cyl.x_mm), float(cyl.y_mm), r_phys, origin, cell_mm
        )
        if cyl_pad > 0:
            _draw_cylinder(
                pad_mask,
                float(cyl.x_mm),
                float(cyl.y_mm),
                r_phys + cyl_pad,
                origin,
                cell_mm,
            )
    if cyl_pad > 0:
        if region_mask is not None:
            pad_mask = cv2.bitwise_and(pad_mask, region_mask)
        cyl_mask = np.maximum(cyl_mask, pad_mask)
    elif region_mask is not None and cylinders:
        # Even with pad=0, keep physical disks; inflate clip uses region below.
        pass

    base = np.maximum(np.maximum(shell_mask, wall_mask), post_mask)
    occ = np.maximum(base, cyl_mask)

    robot_r = float(robot.get("radius_mm", 0.0))
    clearance = float(robot.get("clearance_scale", 1.0))
    inflate_mm = robot_r * clearance
    inflate_cells = int(np.ceil(inflate_mm / cell_mm)) if inflate_mm > 0 else 0
    base_inflated = _inflate(base, inflate_cells, inflate_shape)
    if cylinders:
        cyl_inflated = _inflate(cyl_mask, inflate_cells, inflate_shape)
        if region_mask is not None:
            # Soft clearance from cylinders must not occupy corridors outside
            # the cylinder block (inflate would otherwise spill past the walls).
            cyl_inflated = cv2.bitwise_and(cyl_inflated, region_mask)
            # Always keep the physical disk even if it sits on the rim.
            cyl_inflated = np.maximum(cyl_inflated, cyl_mask)
        inflated = np.maximum(base_inflated, cyl_inflated)
    else:
        inflated = base_inflated

    scale = max(1, int(round(2.0 / cell_mm)))
    preview = cv2.resize(
        (1 - occ) * 255,
        (cols * scale, rows * scale),
        interpolation=cv2.INTER_NEAREST,
    )
    preview_bgr = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)

    # Fine-warp overlay: same geometry as combine, but occupancy colours
    overlay = fine.warped_bgr.copy()
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    ppm = ppc / maze_cell

    def mm_to_px(x_mm: float, y_mm: float) -> tuple[int, int]:
        return (
            int(round((x_mm / maze_cell + margin) * ppc)),
            int(round((y_mm / maze_cell + margin) * ppc)),
        )

    for seg in shell.segments:
        colour = (0, 0, 180)
        pts = np.vstack([seg.inner_mm, seg.outer_mm[::-1]])
        px = np.array([mm_to_px(float(x), float(y)) for x, y in pts], dtype=np.int32)
        cv2.fillConvexPoly(overlay, px, colour)

    wall_px = max(1, int(round(wall_th * ppm)))
    for wall in walls.walls:
        if wall.kind == "h":
            a = poles_mm.get((wall.b0, wall.a))
            b = poles_mm.get((wall.b1, wall.a))
        else:
            a = poles_mm.get((wall.a, wall.b0))
            b = poles_mm.get((wall.a, wall.b1))
        if a is None or b is None:
            continue
        cv2.line(overlay, mm_to_px(*a), mm_to_px(*b), (0, 0, 255), wall_px, cv2.LINE_AA)

    half = post_mm / 2.0
    for x, y in poles_mm.values():
        p0 = mm_to_px(x - half, y - half)
        p1 = mm_to_px(x + half, y + half)
        cv2.rectangle(overlay, p0, p1, (0, 200, 255), thickness=-1)

    # Magenta: physical radius outline + padded disk clipped to the cylinder
    # cell block (same as occupancy clearance).
    region_overlay: np.ndarray | None = None
    if region is not None:
        region_overlay = np.zeros(overlay.shape[:2], dtype=np.uint8)
        x0 = float(region.col0) * maze_cell
        y0 = float(region.row0) * maze_cell
        x1 = float(region.col1 + 1) * maze_cell
        y1 = float(region.row1 + 1) * maze_cell
        cv2.rectangle(
            region_overlay,
            mm_to_px(x0, y0),
            mm_to_px(x1, y1),
            255,
            thickness=-1,
        )

    for cyl in cylinders:
        centre = mm_to_px(float(cyl.x_mm), float(cyl.y_mm))
        r_phys_px = max(1, int(round(float(cyl.radius_mm) * ppm)))
        if cyl_pad > 0:
            r_pad_px = max(1, int(round((float(cyl.radius_mm) + cyl_pad) * ppm)))
            pad_layer = np.zeros(overlay.shape[:2], dtype=np.uint8)
            cv2.circle(
                pad_layer, centre, r_pad_px, 255, thickness=-1, lineType=cv2.LINE_AA
            )
            if region_overlay is not None:
                pad_layer = cv2.bitwise_and(pad_layer, region_overlay)
            tint = overlay.copy()
            tint[pad_layer > 0] = (255, 0, 255)
            overlay = cv2.addWeighted(tint, 0.28, overlay, 0.72, 0)
        cv2.circle(overlay, centre, r_phys_px, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(overlay, centre, 2, (255, 0, 255), -1, cv2.LINE_AA)

    return OccupancyMap(
        occupancy=occ,
        inflated=inflated,
        cell_mm=cell_mm,
        origin_mm=origin,
        width_mm=width_mm,
        height_mm=height_mm,
        overlay_bgr=overlay,
        preview_bgr=preview_bgr,
    )



def load_occupancy(
    output_dir: str | Path,
    *,
    prefix: str = "08_occupancy",
) -> tuple[OccupancyMap, dict]:
    """Reload ``08_occupancy`` npy/yaml written by ``save_occupancy_debug``."""
    out = Path(output_dir)
    occ_path = out / f"{prefix}.npy"
    inf_path = out / f"{prefix}_inflated.npy"
    yml_path = out / f"{prefix}.yaml"
    missing = [p.name for p in (occ_path, inf_path, yml_path) if not p.is_file()]
    if missing:
        raise FileNotFoundError(
            f"missing occupancy {', '.join(missing)}; run the full pipeline first"
        )
    meta = yaml.safe_load(yml_path.read_text(encoding="utf-8")) or {}
    occupancy = np.load(occ_path)
    inflated = np.load(inf_path)
    preview = cv2.imread(str(out / f"{prefix}.png"))
    overlay = cv2.imread(str(out / f"{prefix}_overlay.png"))
    dummy = np.zeros((*occupancy.shape, 3), dtype=np.uint8)
    origin = meta.get("origin_mm") or [0.0, 0.0]
    return (
        OccupancyMap(
            occupancy=occupancy,
            inflated=inflated,
            cell_mm=float(meta["cell_mm"]),
            origin_mm=(float(origin[0]), float(origin[1])),
            width_mm=float(meta.get("width_mm") or occupancy.shape[1] * meta["cell_mm"]),
            height_mm=float(meta.get("height_mm") or occupancy.shape[0] * meta["cell_mm"]),
            overlay_bgr=overlay if overlay is not None else dummy,
            preview_bgr=preview if preview is not None else dummy,
        ),
        meta,
    )


def save_occupancy_debug(
    result: OccupancyMap,
    output_dir: str | Path,
    params: Params,
    *,
    prefix: str = "08_occupancy",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}.png"), result.preview_bgr)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), result.overlay_bgr)

    inflated_preview = cv2.resize(
        (1 - result.inflated) * 255,
        (
            result.inflated.shape[1] * max(1, int(round(2.0 / result.cell_mm))),
            result.inflated.shape[0] * max(1, int(round(2.0 / result.cell_mm))),
        ),
        interpolation=cv2.INTER_NEAREST,
    )
    cv2.imwrite(str(out / f"{prefix}_inflated.png"), inflated_preview)

    np.save(out / f"{prefix}.npy", result.occupancy)
    np.save(out / f"{prefix}_inflated.npy", result.inflated)

    robot = params.robot
    map_cfg = params.map
    payload = {
        "cell_mm": result.cell_mm,
        "origin_mm": [result.origin_mm[0], result.origin_mm[1]],
        "width_mm": result.width_mm,
        "height_mm": result.height_mm,
        "shape_hw": [int(result.occupancy.shape[0]), int(result.occupancy.shape[1])],
        "occupied_cells": int(result.occupancy.sum()),
        "inflated_cells": int(result.inflated.sum()),
        "inflate_shape": str(map_cfg.get("inflate_shape", "diamond")),
        "wall_padding_mm": float(map_cfg.get("wall_padding_mm", 0.0)),
        "post_padding_mm": float(map_cfg.get("post_padding_mm", 0.0)),
        "shell_padding_mm": float(map_cfg.get("shell_padding_mm", 0.0)),
        "cylinder_padding_mm": float(map_cfg.get("cylinder_padding_mm", 0.0)),
        "robot_radius_mm": float(robot.get("radius_mm", 0.0)),
        "clearance_scale": float(robot.get("clearance_scale", 1.0)),
    }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
