from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from poles import Params
from shell import ShellResult, _mm_to_px, _pole_xy_px, _thick_band
from walls import WallDetection, _pole_map, walls_to_yaml_entries
from warp import FineWarpResult


@dataclass
class CombinedWalls:
    overlay_bgr: np.ndarray
    inner_count: int
    shell_count: int
    pole_count: int


def _draw_geometry(
    canvas: np.ndarray,
    fine: FineWarpResult,
    walls: WallDetection,
    shell: ShellResult,
    params: Params,
) -> dict[tuple[int, int], np.ndarray]:
    """Draw shell / inner walls / poles onto canvas. Returns pole map in px."""
    poles = _pole_map(fine, params)
    ppm = float(fine.pixels_per_mm)
    wall_thick = max(1, int(round(float(params.maze["wall_thickness_mm"]) * ppm)))
    post_px = max(2, int(round(float(params.maze["post_size_mm"]) * ppm)))

    for seg in shell.segments:
        colour = (255, 0, 255) if seg.kind == "d" else (0, 140, 255)
        _thick_band(canvas, seg.inner_px, seg.outer_px, colour)

    for w in walls.walls:
        if w.kind == "h":
            p0 = poles.get((w.b0, w.a))
            p1 = poles.get((w.b1, w.a))
        else:
            p0 = poles.get((w.a, w.b0))
            p1 = poles.get((w.a, w.b1))
        if p0 is None or p1 is None:
            continue
        cv2.line(
            canvas,
            (int(round(p0[0])), int(round(p0[1]))),
            (int(round(p1[0])), int(round(p1[1]))),
            (0, 0, 255),
            wall_thick,
            cv2.LINE_AA,
        )

    half = post_px / 2.0
    for (_i, _j), p in poles.items():
        x, y = float(p[0]), float(p[1])
        cv2.rectangle(
            canvas,
            (int(round(x - half)), int(round(y - half))),
            (int(round(x + half)), int(round(y + half))),
            (0, 220, 255),
            thickness=-1,
            lineType=cv2.LINE_AA,
        )
    return poles


def _pt(xy: np.ndarray | tuple[float, float]) -> tuple[int, int]:
    return (int(round(float(xy[0]))), int(round(float(xy[1]))))


def _draw_dim(
    img: np.ndarray,
    a: tuple[float, float] | np.ndarray,
    b: tuple[float, float] | np.ndarray,
    label: str,
    *,
    colour: tuple[int, int, int] = (40, 40, 40),
    offset_px: tuple[float, float] = (0.0, 0.0),
    text_scale: float = 0.42,
) -> None:
    """Dimension line between a and b (true geometry). Label is mid + offset."""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    p0, p1 = _pt((ax, ay)), _pt((bx, by))
    cv2.line(img, p0, p1, colour, 1, cv2.LINE_AA)

    # End ticks perpendicular to the segment.
    dx, dy = bx - ax, by - ay
    length = max(1e-6, (dx * dx + dy * dy) ** 0.5)
    nx, ny = -dy / length * 6.0, dx / length * 6.0
    for px, py in ((ax, ay), (bx, by)):
        cv2.line(
            img,
            _pt((px - nx, py - ny)),
            _pt((px + nx, py + ny)),
            colour,
            1,
            cv2.LINE_AA,
        )

    mid = ((ax + bx) * 0.5, (ay + by) * 0.5)
    tx = mid[0] + offset_px[0]
    ty = mid[1] + offset_px[1]
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, text_scale, 1)
    pad = 3
    x0, y0 = int(tx - tw * 0.5) - pad, int(ty + th * 0.35) - th - pad
    x1, y1 = int(tx + tw * 0.5) + pad, int(ty + th * 0.35) + pad
    cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), thickness=-1)
    cv2.rectangle(img, (x0, y0), (x1, y1), colour, thickness=1, lineType=cv2.LINE_AA)
    cv2.putText(
        img,
        label,
        (int(tx - tw * 0.5), int(ty + th * 0.35)),
        cv2.FONT_HERSHEY_SIMPLEX,
        text_scale,
        colour,
        1,
        cv2.LINE_AA,
    )


def _annotate_measurements(
    img: np.ndarray,
    fine: FineWarpResult,
    shell: ShellResult,
    params: Params,
) -> None:
    maze = params.maze
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    cell_mm = float(maze["cell_size_mm"])
    wall_th = float(maze["wall_thickness_mm"])
    post_mm = float(maze["post_size_mm"])

    meta = shell.meta
    li, ti, ri, bi = [float(v) for v in meta["inner_box_mm"]]
    edge_gap = float(meta.get("derived_edge_gap_mm", meta["outer_pole_to_wall_mm"]))
    corner_gap = float(
        meta.get("derived_corner_gap_mm", meta["corner_pole_to_diagonal_mm"])
    )
    inner_span = float(meta.get("derived_inner_span_mm", ri - li))

    # Prefer actual diagonal segment geometry for leg/hyp labels (TL) and
    # corner-gap callout (BR, so it sits bottom-right with the overview box).
    tl = next((s for s in shell.segments if s.kind == "d" and s.corner == "TL"), None)
    br = next((s for s in shell.segments if s.kind == "d" and s.corner == "BR"), None)
    if tl is not None:
        a, b = tl.inner_mm[0], tl.inner_mm[1]
        left_pt = a if abs(float(a[0]) - li) < abs(float(b[0]) - li) else b
        top_pt = a if abs(float(a[1]) - ti) < abs(float(b[1]) - ti) else b
        corner = np.array([li, ti], dtype=np.float64)
        leg_v = float(np.linalg.norm(left_pt - corner))
        leg_h = float(np.linalg.norm(top_pt - corner))
        hyp = float(np.linalg.norm(left_pt - top_pt))
        tl_left_mm, tl_top_mm = left_pt, top_pt
    else:
        leg_v = leg_h = float(meta["diagonal_leg_mm"])
        hyp = float(meta["diagonal_hypotenuse_mm"])
        tl_left_mm = np.array([li, ti + leg_v], dtype=np.float64)
        tl_top_mm = np.array([li + leg_h, ti], dtype=np.float64)

    def mm_pt(xy: np.ndarray | tuple[float, float]) -> np.ndarray:
        return _mm_to_px(
            np.asarray(xy, dtype=np.float64).reshape(2), margin, ppc, cell_mm
        )

    # --- Shell inner clear (top side, offset outward) ---
    _draw_dim(
        img,
        mm_pt((li, ti)),
        mm_pt((ri, ti)),
        f"inner {inner_span:.0f} mm",
        colour=(0, 90, 180),
        offset_px=(0.0, -28.0),
    )

    # --- Diagonal TL triangle: leg along left, leg along top, hyp ---
    _draw_dim(
        img,
        mm_pt((li, ti)),
        mm_pt(tl_left_mm),
        f"leg {leg_v:.0f}",
        colour=(180, 0, 180),
        offset_px=(-34.0, 0.0),
    )
    _draw_dim(
        img,
        mm_pt((li, ti)),
        mm_pt(tl_top_mm),
        f"leg {leg_h:.0f}",
        colour=(180, 0, 180),
        offset_px=(0.0, -18.0),
    )
    _draw_dim(
        img,
        mm_pt(tl_left_mm),
        mm_pt(tl_top_mm),
        f"hyp {hyp:.0f}",
        colour=(180, 0, 180),
        offset_px=(14.0, 14.0),
    )

    # --- Edge pole → shell (left) ---
    pole0 = _pole_xy_px(0, 4, margin, ppc)
    shell_hit = mm_pt((li, 4.0 * cell_mm))
    _draw_dim(
        img,
        shell_hit,
        pole0,
        f"edge gap {edge_gap:.0f}",
        colour=(0, 128, 0),
        offset_px=(0.0, -16.0),
    )

    # --- Corner pole → BR diagonal (bottom-right) ---
    # Plane distance from near-corner pole centre (ci,cj) to inner diag face.
    cols = int(maze["pole_grid_cols"])
    rows = int(maze["pole_grid_rows"])
    ci, cj = cols - 2, rows - 2  # mirror of (1,1) on i+j=2 ring
    # Ideal lattice (same frame as shell mm) so the segment is exactly the
    # configured plane distance.
    pole_br = _pole_xy_px(ci, cj, margin, ppc)
    if br is not None:
        s_br = float(br.inner_mm[0][0] + br.inner_mm[0][1])
        px = float(ci) * cell_mm
        py = float(cj) * cell_mm
        delta = (px + py - s_br) / 2.0
        foot = mm_pt((px - delta, py - delta))
    else:
        foot = mm_pt(
            (ri - corner_gap / (2.0**0.5), bi - corner_gap / (2.0**0.5))
        )
    _draw_dim(
        img,
        foot,
        pole_br,
        f"corner gap {corner_gap:.0f}",
        colour=(0, 128, 0),
        offset_px=(28.0, 22.0),
    )
    cv2.circle(img, _pt(pole_br), 3, (0, 128, 0), -1, cv2.LINE_AA)
    cv2.circle(img, _pt(foot), 3, (0, 128, 0), -1, cv2.LINE_AA)

    # --- Cell size between poles (0,0)→(1,0) ---
    p00 = _pole_xy_px(0, 0, margin, ppc)
    p10 = _pole_xy_px(1, 0, margin, ppc)
    _draw_dim(
        img,
        p00,
        p10,
        f"cell {cell_mm:.0f}",
        colour=(30, 30, 30),
        offset_px=(0.0, 22.0),
    )

    # --- Wall thickness + post size callouts ---
    sample_y = 3.0 * cell_mm
    half_th = wall_th * 0.5
    _draw_dim(
        img,
        mm_pt((2.0 * cell_mm, sample_y - half_th)),
        mm_pt((2.0 * cell_mm, sample_y + half_th)),
        f"wall th {wall_th:.0f}",
        colour=(0, 0, 200),
        offset_px=(40.0, 0.0),
    )

    half_post = post_mm * 0.5
    post_c = mm_pt((3.0 * cell_mm, 3.0 * cell_mm))
    scale = ppc / cell_mm
    _draw_dim(
        img,
        (post_c[0] - half_post * scale, post_c[1]),
        (post_c[0] + half_post * scale, post_c[1]),
        f"post {post_mm:.0f}",
        colour=(0, 160, 200),
        offset_px=(0.0, 18.0),
    )

    lines = [
        f"shell inner clear: {inner_span:.0f} mm",
        f"diagonal: {leg_h:.0f} x {leg_v:.0f} (hyp {hyp:.0f})",
        f"edge gap: {edge_gap:.0f} mm  |  corner gap: {corner_gap:.0f} mm",
        f"cell: {cell_mm:.0f} mm  |  wall th: {wall_th:.0f} mm  |  post: {post_mm:.0f} mm",
    ]
    text_scale = 0.45
    thickness = 1
    sizes = [
        cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, text_scale, thickness)[0]
        for t in lines
    ]
    tw = max(s[0] for s in sizes) + 16
    th_box = sum(s[1] for s in sizes) + 8 * len(lines) + 8
    # Overview box — bottom-right
    margin_box = 12
    x0 = img.shape[1] - tw - margin_box
    y0 = img.shape[0] - th_box - margin_box
    cv2.rectangle(img, (x0, y0), (x0 + tw, y0 + th_box), (255, 255, 255), thickness=-1)
    cv2.rectangle(img, (x0, y0), (x0 + tw, y0 + th_box), (40, 40, 40), 1, cv2.LINE_AA)
    y = y0 + 18
    for t, (_w, h) in zip(lines, sizes):
        cv2.putText(
            img,
            t,
            (x0 + 8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            text_scale,
            (30, 30, 30),
            thickness,
            cv2.LINE_AA,
        )
        y += h + 8


def combine_walls(
    fine: FineWarpResult,
    walls: WallDetection,
    shell: ShellResult,
    params: Params,
) -> CombinedWalls:
    """Overlay shell, present inner walls, and poles (no absent-edge markers)."""
    overlay = fine.warped_bgr.copy()
    poles = _draw_geometry(overlay, fine, walls, shell, params)
    return CombinedWalls(
        overlay_bgr=overlay,
        inner_count=len(walls.walls),
        shell_count=len(shell.segments),
        pole_count=len(poles),
    )


def save_combined_walls(
    combined: CombinedWalls,
    walls: WallDetection,
    shell: ShellResult,
    fine: FineWarpResult,
    output_dir: str | Path,
    params: Params,
    *,
    prefix: str = "07_all_walls",
    cylinders: list | None = None,
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), combined.overlay_bgr)

    if cylinders:
        from cylinders import append_region_cells_yaml, paint_cylinder_region_on

        region_overlay, region = paint_cylinder_region_on(
            combined.overlay_bgr, fine, cylinders, params, walls=walls
        )
        cv2.imwrite(str(out / f"{prefix}_overlay_region.png"), region_overlay)
        if region is not None:
            cyl_prefix = str(params.output.get("cylinders_prefix", "05b_cylinders"))
            append_region_cells_yaml(out, region, cylinders_prefix=cyl_prefix)

    # Photo background + measurement callouts.
    measured_overlay = combined.overlay_bgr.copy()
    _annotate_measurements(measured_overlay, fine, shell, params)
    cv2.imwrite(str(out / f"{prefix}_overlay_measured.png"), measured_overlay)

    # Schematic (no photo) + measurement callouts.
    schematic = np.full_like(combined.overlay_bgr, 245)
    _draw_geometry(schematic, fine, walls, shell, params)
    _annotate_measurements(schematic, fine, shell, params)
    cv2.imwrite(str(out / f"{prefix}_schematic_measured.png"), schematic)

    maze = params.maze
    poles = _pole_map(fine, params)
    cell_mm = float(maze["cell_size_mm"])
    payload = {
        "rows": int(maze["grid_rows"]),
        "cols": int(maze["grid_cols"]),
        "cell_size_mm": cell_mm,
        "post_size_mm": float(maze["post_size_mm"]),
        "wall_thickness_mm": float(maze["wall_thickness_mm"]),
        "poles": [
            {"i": int(i), "j": int(j), "x_mm": float(i) * cell_mm, "y_mm": float(j) * cell_mm}
            for (i, j) in sorted(poles)
        ],
        "inner_walls": walls_to_yaml_entries(walls.walls),
        "shell": {
            **{k: v for k, v in shell.meta.items() if k != "segments"},
            "segments": [
                {
                    "kind": s.kind,
                    "corner": s.corner,
                    "side": s.side,
                    "inner_mm": s.inner_mm.tolist(),
                    "outer_mm": s.outer_mm.tolist(),
                }
                for s in shell.segments
            ],
        },
    }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
