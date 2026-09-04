from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from poles import Params
from warp import FineWarpResult


@dataclass
class ShellSeg:
    kind: str  # "h" | "v" | "d"
    corner: str | None  # TL/TR/BR/BL for diagonals
    side: str | None  # top/right/bottom/left for straight
    inner_mm: np.ndarray  # (2, 2) endpoints, mm (origin at pole 0,0)
    outer_mm: np.ndarray
    inner_px: np.ndarray
    outer_px: np.ndarray


@dataclass
class ShellResult:
    segments: list[ShellSeg]
    overlay_bgr: np.ndarray
    meta: dict


def _pole_xy_px(i: int, j: int, margin: float, ppc: float) -> np.ndarray:
    return np.array([(i + margin) * ppc, (j + margin) * ppc], dtype=np.float64)


def _mm_to_px(pts_mm: np.ndarray, margin: float, ppc: float, cell_mm: float) -> np.ndarray:
    origin = np.array([margin * ppc, margin * ppc], dtype=np.float64)
    return origin + pts_mm * (ppc / cell_mm)


def _thick_band(
    overlay: np.ndarray,
    inner: np.ndarray,
    outer: np.ndarray,
    colour: tuple[int, int, int],
) -> None:
    poly = np.vstack([inner, outer[::-1]]).astype(np.int32)
    cv2.fillConvexPoly(overlay, poly, colour, lineType=cv2.LINE_AA)
    cv2.polylines(overlay, [inner.astype(np.int32)], False, (0, 0, 0), 1, cv2.LINE_AA)


def build_shell(fine: FineWarpResult, params: Params) -> ShellResult:
    """Place outer shell (4 straight + 4 diagonal walls) in the fine-warp frame.

    Replace `shell:` values in param.yaml with measured mm. Placement modes
    (first match wins):

    - inner_to_inner_mm: inner faces from measured clear opening, centred on
      the pole grid; outer faces = inner ± thickness (thickness is cosmetic
      for LiDAR — rays hit the inner faces).
    - use_outer_to_outer: outer faces from overall OD; inner = outer ± th.
    - else: inner faces from outer_pole_to_wall_mm off the outer pole ring.

    Diagonals:
    - use_diagonal_leg: cut length along faces from square corners (leg / hyp).
      When placing from inner_to_inner_mm, leg/hyp are the *inner* triangle.
    - else: offset from the three near-corner poles (i+j=2 and mirrors) by
      corner_pole_to_diagonal_mm, clipped to the straight faces.
    """
    maze = params.maze
    cfg = params.shell
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    cell_mm = float(maze["cell_size_mm"])
    ppm = float(fine.pixels_per_mm)
    cols = int(maze["pole_grid_cols"])
    rows = int(maze["pole_grid_rows"])

    gap = float(cfg["outer_pole_to_wall_mm"])
    t_out = float(cfg["outer_wall_thickness_mm"])
    t_diag = float(cfg["diagonal_thickness_mm"])
    d_diag = float(cfg["corner_pole_to_diagonal_mm"])
    outer_to_outer = float(cfg["outer_to_outer_mm"])
    inner_to_inner = cfg.get("inner_to_inner_mm")
    if inner_to_inner is not None:
        inner_to_inner = float(inner_to_inner)

    leg_mm = cfg.get("diagonal_leg_mm")
    hyp_mm = cfg.get("diagonal_hypotenuse_mm")
    if leg_mm is not None:
        leg_mm = float(leg_mm)
    elif hyp_mm is not None:
        leg_mm = float(hyp_mm) / math.sqrt(2.0)
    else:
        leg_mm = 250.0
    hyp_mm = float(hyp_mm) if hyp_mm is not None else leg_mm * math.sqrt(2.0)

    xmax = (cols - 1) * cell_mm
    ymax = (rows - 1) * cell_mm
    centre = xmax / 2.0

    if inner_to_inner is not None:
        half = inner_to_inner / 2.0
        left_inner_mm = centre - half
        right_inner_mm = centre + half
        top_inner_mm = centre - half
        bottom_inner_mm = centre + half
        left_outer_mm = left_inner_mm - t_out
        right_outer_mm = right_inner_mm + t_out
        top_outer_mm = top_inner_mm - t_out
        bottom_outer_mm = bottom_inner_mm + t_out
    elif bool(cfg.get("use_outer_to_outer", True)):
        half = outer_to_outer / 2.0
        left_outer_mm = centre - half
        right_outer_mm = centre + half
        top_outer_mm = centre - half
        bottom_outer_mm = centre + half
        left_inner_mm = left_outer_mm + t_out
        right_inner_mm = right_outer_mm - t_out
        top_inner_mm = top_outer_mm + t_out
        bottom_inner_mm = bottom_outer_mm - t_out
    else:
        left_inner_mm = -gap
        right_inner_mm = xmax + gap
        top_inner_mm = -gap
        bottom_inner_mm = ymax + gap
        left_outer_mm = left_inner_mm - t_out
        right_outer_mm = right_inner_mm + t_out
        top_outer_mm = top_inner_mm - t_out
        bottom_outer_mm = bottom_inner_mm + t_out

    use_leg = bool(cfg.get("use_diagonal_leg", False))
    segments: list[ShellSeg] = []

    def add_seg(
        kind: str,
        *,
        corner: str | None,
        side: str | None,
        inner_a: np.ndarray,
        inner_b: np.ndarray,
        outer_a: np.ndarray,
        outer_b: np.ndarray,
    ) -> None:
        inner = np.vstack([inner_a, inner_b])
        outer = np.vstack([outer_a, outer_b])
        segments.append(
            ShellSeg(
                kind=kind,
                corner=corner,
                side=side,
                inner_mm=inner,
                outer_mm=outer,
                inner_px=_mm_to_px(inner, margin, ppc, cell_mm),
                outer_px=_mm_to_px(outer, margin, ppc, cell_mm),
            )
        )

    li, ri = left_inner_mm, right_inner_mm
    ti, bi = top_inner_mm, bottom_inner_mm
    lo, ro = left_outer_mm, right_outer_mm
    to, bo = top_outer_mm, bottom_outer_mm

    if use_leg:
        L = leg_mm
        # Inner leg/hyp → outer cut for a parallel band of thickness t_diag.
        # Fallback when thickness is unused: L_out ≈ L.
        L_out = L + 2.0 * t_out - t_diag * math.sqrt(2.0)
        if L_out < 1.0:
            L_out = hyp_mm / math.sqrt(2.0)

        add_seg(
            "d",
            corner="TL",
            side=None,
            inner_a=np.array([li, ti + L]),
            inner_b=np.array([li + L, ti]),
            outer_a=np.array([lo, to + L_out]),
            outer_b=np.array([lo + L_out, to]),
        )
        add_seg(
            "d",
            corner="TR",
            side=None,
            inner_a=np.array([ri - L, ti]),
            inner_b=np.array([ri, ti + L]),
            outer_a=np.array([ro - L_out, to]),
            outer_b=np.array([ro, to + L_out]),
        )
        add_seg(
            "d",
            corner="BR",
            side=None,
            inner_a=np.array([ri, bi - L]),
            inner_b=np.array([ri - L, bi]),
            outer_a=np.array([ro, bo - L_out]),
            outer_b=np.array([ro - L_out, bo]),
        )
        add_seg(
            "d",
            corner="BL",
            side=None,
            inner_a=np.array([li + L, bi]),
            inner_b=np.array([li, bi - L]),
            outer_a=np.array([lo + L_out, bo]),
            outer_b=np.array([lo, bo - L_out]),
        )
        add_seg(
            "h",
            corner=None,
            side="top",
            inner_a=np.array([li + L, ti]),
            inner_b=np.array([ri - L, ti]),
            outer_a=np.array([lo + L_out, to]),
            outer_b=np.array([ro - L_out, to]),
        )
        add_seg(
            "v",
            corner=None,
            side="right",
            inner_a=np.array([ri, ti + L]),
            inner_b=np.array([ri, bi - L]),
            outer_a=np.array([ro, to + L_out]),
            outer_b=np.array([ro, bo - L_out]),
        )
        add_seg(
            "h",
            corner=None,
            side="bottom",
            inner_a=np.array([ri - L, bi]),
            inner_b=np.array([li + L, bi]),
            outer_a=np.array([ro - L_out, bo]),
            outer_b=np.array([lo + L_out, bo]),
        )
        add_seg(
            "v",
            corner=None,
            side="left",
            inner_a=np.array([li, bi - L]),
            inner_b=np.array([li, ti + L]),
            outer_a=np.array([lo, bo - L_out]),
            outer_b=np.array([lo, to + L_out]),
        )
    else:
        # Near-corner poles sit on i+j=2 (and mirrors). Offset toward each
        # corner by d_diag (plane distance), then by diagonal thickness.
        off = d_diag * math.sqrt(2.0)
        c2 = 2.0 * cell_mm

        def tl_sum(inner: bool) -> float:
            o = off if inner else off + t_diag * math.sqrt(2.0)
            return c2 - o

        def tr_k(inner: bool) -> float:
            # (xmax - x) + y = c2 - o  →  -x + y = c2 - o - xmax
            o = off if inner else off + t_diag * math.sqrt(2.0)
            return c2 - o - xmax

        def br_sum(inner: bool) -> float:
            o = off if inner else off + t_diag * math.sqrt(2.0)
            return xmax + ymax - (c2 - o)

        def bl_k(inner: bool) -> float:
            # x + (ymax - y) = c2 - o  →  x - y = c2 - o - ymax
            o = off if inner else off + t_diag * math.sqrt(2.0)
            return c2 - o - ymax

        def hit_tl(s: float, xw: float, yw: float) -> tuple[np.ndarray, np.ndarray]:
            return np.array([xw, s - xw]), np.array([s - yw, yw])

        def hit_tr(k: float, xw: float, yw: float) -> tuple[np.ndarray, np.ndarray]:
            # y = x + k
            return np.array([xw, xw + k]), np.array([yw - k, yw])

        def hit_br(s: float, xw: float, yw: float) -> tuple[np.ndarray, np.ndarray]:
            return np.array([xw, s - xw]), np.array([s - yw, yw])

        def hit_bl(k: float, xw: float, yw: float) -> tuple[np.ndarray, np.ndarray]:
            # x = y + k
            return np.array([xw, xw - k]), np.array([yw + k, yw])

        tl_i, tl_o = hit_tl(tl_sum(True), li, ti), hit_tl(tl_sum(False), lo, to)
        tr_i, tr_o = hit_tr(tr_k(True), ri, ti), hit_tr(tr_k(False), ro, to)
        br_i, br_o = hit_br(br_sum(True), ri, bi), hit_br(br_sum(False), ro, bo)
        bl_i, bl_o = hit_bl(bl_k(True), li, bi), hit_bl(bl_k(False), lo, bo)

        add_seg("d", corner="TL", side=None, inner_a=tl_i[0], inner_b=tl_i[1], outer_a=tl_o[0], outer_b=tl_o[1])
        add_seg("d", corner="TR", side=None, inner_a=tr_i[1], inner_b=tr_i[0], outer_a=tr_o[1], outer_b=tr_o[0])
        add_seg("d", corner="BR", side=None, inner_a=br_i[0], inner_b=br_i[1], outer_a=br_o[0], outer_b=br_o[1])
        add_seg("d", corner="BL", side=None, inner_a=bl_i[1], inner_b=bl_i[0], outer_a=bl_o[1], outer_b=bl_o[0])

        add_seg("h", corner=None, side="top", inner_a=tl_i[1], inner_b=tr_i[1], outer_a=tl_o[1], outer_b=tr_o[1])
        add_seg("v", corner=None, side="right", inner_a=tr_i[0], inner_b=br_i[0], outer_a=tr_o[0], outer_b=br_o[0])
        add_seg("h", corner=None, side="bottom", inner_a=br_i[1], inner_b=bl_i[1], outer_a=br_o[1], outer_b=bl_o[1])
        add_seg("v", corner=None, side="left", inner_a=bl_i[0], inner_b=tl_i[0], outer_a=bl_o[0], outer_b=tl_o[0])

    overlay = fine.warped_bgr.copy()
    for seg in segments:
        colour = (255, 0, 255) if seg.kind == "d" else (0, 140, 255)
        _thick_band(overlay, seg.inner_px, seg.outer_px, colour)

    # Reference poles used for the i+j=2 diagonal offset (and mirrors)
    refs = [
        (2, 0),
        (1, 1),
        (0, 2),
        (cols - 3, 0),
        (cols - 2, 1),
        (cols - 1, 2),
        (cols - 1, rows - 3),
        (cols - 2, rows - 2),
        (cols - 3, rows - 1),
        (0, rows - 3),
        (1, rows - 2),
        (2, rows - 1),
    ]
    for i, j in refs:
        if 0 <= i < cols and 0 <= j < rows:
            p = _pole_xy_px(i, j, margin, ppc)
            cv2.circle(
                overlay,
                (int(round(p[0])), int(round(p[1]))),
                5,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    # Pole-ring → shell checks (maze frame: poles at 0..xmax / 0..ymax).
    derived_edge_gap_mm = -left_inner_mm  # == right_inner_mm - xmax when centred
    # Mid near-corner pole (1,1): plane distance to TL inner diagonal x+y = li+ti+L.
    if use_leg:
        tl_sum = left_inner_mm + top_inner_mm + leg_mm
        derived_corner_gap_mm = abs(2.0 * cell_mm - tl_sum) / math.sqrt(2.0)
    else:
        derived_corner_gap_mm = d_diag

    meta = {
        "inner_to_inner_mm": inner_to_inner,
        "outer_to_outer_mm": outer_to_outer,
        "derived_inner_span_mm": right_inner_mm - left_inner_mm,
        "derived_outer_span_mm": right_outer_mm - left_outer_mm,
        "outer_wall_thickness_mm": t_out,
        "diagonal_thickness_mm": t_diag,
        "outer_pole_to_wall_mm": gap,
        "corner_pole_to_diagonal_mm": d_diag,
        "derived_edge_gap_mm": derived_edge_gap_mm,
        "derived_corner_gap_mm": derived_corner_gap_mm,
        "diagonal_leg_mm": leg_mm,
        "diagonal_hypotenuse_mm": hyp_mm,
        "use_outer_to_outer": bool(cfg.get("use_outer_to_outer", True)),
        "use_diagonal_leg": use_leg,
        "pixels_per_mm": ppm,
        "inner_box_mm": [left_inner_mm, top_inner_mm, right_inner_mm, bottom_inner_mm],
        "outer_box_mm": [left_outer_mm, top_outer_mm, right_outer_mm, bottom_outer_mm],
    }
    return ShellResult(segments=segments, overlay_bgr=overlay, meta=meta)


def load_shell_from_yaml(
    path: str | Path,
    *,
    overlay_bgr: np.ndarray | None = None,
) -> ShellResult:
    """Read ``06_shell.yaml`` written by ``save_shell_debug``."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    segments: list[ShellSeg] = []
    for raw in data.get("segments") or []:
        segments.append(
            ShellSeg(
                kind=str(raw["kind"]),
                corner=raw.get("corner"),
                side=raw.get("side"),
                inner_mm=np.asarray(raw["inner_mm"], dtype=np.float64),
                outer_mm=np.asarray(raw["outer_mm"], dtype=np.float64),
                inner_px=np.zeros((2, 2), dtype=np.float64),
                outer_px=np.zeros((2, 2), dtype=np.float64),
            )
        )
    meta = {k: v for k, v in data.items() if k != "segments"}
    dummy = overlay_bgr if overlay_bgr is not None else np.zeros((1, 1, 3), dtype=np.uint8)
    return ShellResult(segments=segments, overlay_bgr=dummy, meta=meta)


def save_shell_debug(
    result: ShellResult,
    output_dir: str | Path,
    *,
    prefix: str = "06_shell",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), result.overlay_bgr)

    payload = {
        **result.meta,
        "segments": [
            {
                "kind": s.kind,
                "corner": s.corner,
                "side": s.side,
                "inner_mm": s.inner_mm.tolist(),
                "outer_mm": s.outer_mm.tolist(),
            }
            for s in result.segments
        ],
    }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)
