from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from poles import Params, chamfer_missing_poles
from warp import FineWarpResult


@dataclass
class WallSeg:
    kind: str  # "h" or "v"
    a: int
    b0: int
    b1: int


@dataclass
class WallDetection:
    walls: list[WallSeg]
    scores: list[tuple[str, int, int, float, float]]
    overlay_bgr: np.ndarray


def _pole_map(fine: FineWarpResult, params: Params) -> dict[tuple[int, int], np.ndarray]:
    if not bool(params.poles.get("assume_all_present", False)):
        return {
            (int(i), int(j)): fine.poles_dst_px[k]
            for k, (i, j) in enumerate(fine.poles_ij)
        }

    cols = int(params.maze["pole_grid_cols"])
    rows = int(params.maze["pole_grid_rows"])
    margin = float(params.warp["fine"]["margin_cells"])
    ppc = float(fine.px_per_cell)
    skip = (
        chamfer_missing_poles(cols, rows)
        if bool(params.poles.get("skip_chamfer_poles", True))
        else set()
    )

    poles: dict[tuple[int, int], np.ndarray] = {}
    for i in range(cols):
        for j in range(rows):
            if (i, j) in skip:
                continue
            poles[(i, j)] = np.array(
                [(i + margin) * ppc, (j + margin) * ppc],
                dtype=np.float32,
            )
    return poles


def _edge_geometry(
    p0: np.ndarray,
    p1: np.ndarray,
    end_trim: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    p0 = np.asarray(p0, dtype=np.float64)
    p1 = np.asarray(p1, dtype=np.float64)
    length = float(np.linalg.norm(p1 - p0))
    n = max(12, int(round(length)))
    ts = np.linspace(end_trim, 1.0 - end_trim, n)
    pts = p0 + np.outer(ts, (p1 - p0))
    direction = p1 - p0
    direction /= np.linalg.norm(direction) + 1e-6
    perp = np.array([-direction[1], direction[0]], dtype=np.float64)
    return pts, direction, perp


def _ridge_dark_score(
    gray: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    *,
    half_width: int,
    end_trim: float,
    body_percentile: float = 70.0,
) -> float:
    """Mean of the darkest stations along the edge (catches thin / glossy walls).

    ``body_percentile`` keeps only the darkest fraction of per-station minima
    before averaging. 70 is the primary ridge score; a lower tail (e.g. 40) is
    used separately to catch glossy walls with only a dark segment.
    """
    pts, _direction, perp = _edge_geometry(p0, p1, end_trim)
    h, w = gray.shape
    mins: list[float] = []
    for x, y in pts:
        vals: list[float] = []
        for offset in range(-half_width, half_width + 1):
            q = np.array([x, y]) + offset * perp
            xi = int(round(q[0]))
            yi = int(round(q[1]))
            if 0 <= xi < w and 0 <= yi < h:
                vals.append(float(gray[yi, xi]))
        if vals:
            mins.append(min(vals))
    if not mins:
        return 255.0
    mins_arr = np.asarray(mins, dtype=np.float64)
    pct = float(np.clip(body_percentile, 1.0, 100.0))
    cut = float(np.percentile(mins_arr, pct))
    body = mins_arr[mins_arr <= cut]
    return float(body.mean()) if len(body) else float(mins_arr.mean())


def _line_enhance(gray: np.ndarray, kernel_len: int) -> np.ndarray:
    """Highlight thin dark or bright lines (matte or glossy acrylic)."""
    k_h = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_len, 1))
    k_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, kernel_len))
    black = cv2.max(
        cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, k_h),
        cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, k_v),
    )
    white = cv2.max(
        cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k_h),
        cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k_v),
    )
    return cv2.max(black, white)


def _enhance_score(
    enhance: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    *,
    half_width: int,
    end_trim: float,
) -> float:
    pts, _direction, perp = _edge_geometry(p0, p1, end_trim)
    h, w = enhance.shape
    vals: list[float] = []
    for x, y in pts:
        for offset in range(-half_width, half_width + 1):
            q = np.array([x, y]) + offset * perp
            xi = int(round(q[0]))
            yi = int(round(q[1]))
            if 0 <= xi < w and 0 <= yi < h:
                vals.append(float(enhance[yi, xi]))
    if not vals:
        return 0.0
    return float(np.percentile(vals, 75))


def _blank_cylinders_gray(
    gray: np.ndarray,
    cylinders: list,
    *,
    floor_gray: float = 200.0,
    radius_scale: float = 1.15,
) -> np.ndarray:
    """Paint cylinder disks bright so they cannot fake a dark wall ridge."""
    if not cylinders:
        return gray
    out = gray.copy()
    fill = int(np.clip(floor_gray, 0, 255))
    for c in cylinders:
        wx, wy = c.warp_xy
        r = max(1, int(round(float(c.radius_warp_px) * radius_scale)))
        cv2.circle(
            out,
            (int(round(wx)), int(round(wy))),
            r,
            fill,
            thickness=-1,
            lineType=cv2.LINE_AA,
        )
    return out


def detect_walls(
    fine: FineWarpResult,
    params: Params,
    *,
    cylinders: list | None = None,
) -> WallDetection:
    cfg = params.walls
    half_width = int(cfg["sample_half_width_px"])
    end_trim = float(cfg["end_trim"])
    dark_thr = float(cfg["dark_ridge_threshold"])
    enhance_thr = float(cfg["enhance_threshold"])
    enhance_strong = float(cfg.get("enhance_strong_threshold", enhance_thr + 2.0))
    enhance_max_tail = float(cfg.get("enhance_max_dark_tail", 126.0))
    kernel_len = int(cfg["enhance_kernel_px"])
    body_pct = float(cfg.get("dark_body_percentile", 70.0))
    tail_pct = float(cfg.get("dark_tail_percentile", 40.0))
    tail_thr = float(cfg.get("dark_tail_threshold", 112.0))

    gray = cv2.cvtColor(fine.warped_bgr, cv2.COLOR_BGR2GRAY)
    if cylinders:
        gray = _blank_cylinders_gray(
            gray,
            cylinders,
            floor_gray=float(cfg.get("cylinder_blank_gray", 200.0)),
            radius_scale=float(cfg.get("cylinder_blank_radius_scale", 1.15)),
        )
    enhance = _line_enhance(gray, kernel_len)
    poles = _pole_map(fine, params)

    walls: list[WallSeg] = []
    scores: list[tuple[str, int, int, float, float]] = []
    overlay = fine.warped_bgr.copy()

    for (i, j), p0 in poles.items():
        for di, dj, kind in ((1, 0, "h"), (0, 1, "v")):
            key = (i + di, j + dj)
            if key not in poles:
                continue
            p1 = poles[key]
            dark = _ridge_dark_score(
                gray,
                p0,
                p1,
                half_width=half_width,
                end_trim=end_trim,
                body_percentile=body_pct,
            )
            dark_tail = (
                dark
                if abs(tail_pct - body_pct) < 1e-9
                else _ridge_dark_score(
                    gray,
                    p0,
                    p1,
                    half_width=half_width,
                    end_trim=end_trim,
                    body_percentile=tail_pct,
                )
            )
            enh = _enhance_score(
                enhance,
                p0,
                p1,
                half_width=max(1, half_width // 2),
                end_trim=end_trim,
            )
            scores.append((kind, i, j, dark, enh))
            # Primary dark ridge, strong enhance, or a dark *tail* (partial
            # glossy wall). Weak enhance (~threshold) also needs a dark enough
            # tail so empty right-rail edges (cylinder13 h@8,5) stay out.
            present = (
                dark <= dark_thr
                or dark_tail <= tail_thr
                or enh >= enhance_strong
                or (enh >= enhance_thr and dark_tail <= enhance_max_tail)
            )

            colour = (0, 0, 255) if present else (0, 200, 0)
            cv2.line(
                overlay,
                (int(round(p0[0])), int(round(p0[1]))),
                (int(round(p1[0])), int(round(p1[1]))),
                colour,
                2,
                cv2.LINE_AA,
            )
            if not present:
                continue

            if kind == "h":
                walls.append(WallSeg(kind="h", a=j, b0=i, b1=i + 1))
            else:
                walls.append(WallSeg(kind="v", a=i, b0=j, b1=j + 1))

    walls.sort(key=lambda w: (w.kind, w.a, w.b0, w.b1))
    return WallDetection(walls=walls, scores=scores, overlay_bgr=overlay)


def walls_to_yaml_entries(walls: list[WallSeg]) -> list[dict]:
    return [{w.kind: [w.a, w.b0, w.b1]} for w in walls]


def load_walls_from_yaml(path: str | Path) -> list[WallSeg]:
    """Read ``05_walls.yaml`` written by ``save_walls_debug``."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    walls: list[WallSeg] = []
    for entry in data.get("walls") or []:
        if "h" in entry:
            a, b0, b1 = entry["h"]
            walls.append(WallSeg(kind="h", a=int(a), b0=int(b0), b1=int(b1)))
        elif "v" in entry:
            a, b0, b1 = entry["v"]
            walls.append(WallSeg(kind="v", a=int(a), b0=int(b0), b1=int(b1)))
    return walls


def load_wall_detection(path: str | Path) -> WallDetection:
    """Walls only; overlay/scores are unused when skipping vision."""
    dummy = np.zeros((1, 1, 3), dtype=np.uint8)
    return WallDetection(
        walls=load_walls_from_yaml(path),
        scores=[],
        overlay_bgr=dummy,
    )


def save_walls_debug(
    detection: WallDetection,
    output_dir: str | Path,
    params: Params,
    *,
    prefix: str = "05_walls",
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}_overlay.png"), detection.overlay_bgr)

    maze = params.maze
    payload = {
        "rows": int(maze["grid_rows"]),
        "cols": int(maze["grid_cols"]),
        "cell_size_mm": float(maze["cell_size_mm"]),
        "post_size_mm": float(maze["post_size_mm"]),
        "wall_thickness_mm": float(maze["wall_thickness_mm"]),
        "walls": walls_to_yaml_entries(detection.walls),
    }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)

    with open(out / f"{prefix}_scores.txt", "w", encoding="utf-8") as f:
        f.write("kind i j dark_ridge enhance\n")
        for kind, i, j, dark, enh in detection.scores:
            f.write(f"{kind} {i} {j} {dark:.3f} {enh:.3f}\n")
