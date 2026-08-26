from __future__ import annotations

import math
from pathlib import Path

from plan_path import PlannedPath, Waypoint, _cell_center_mm
from poles import Params
from shell import ShellResult
from walls import WallDetection, WallSeg, _pole_map
from warp import FineWarpResult


def _fmt(value: float) -> str:
    return f"{value:.4f}f"


def waypoints_robot_m(
    waypoints: list[Waypoint],
    origin_mm: tuple[float, float],
) -> list[tuple[float, float]]:
    """Maze mm (x right, y down) → robot metres (x left, y down), relative to start."""
    ox, oy = origin_mm
    out: list[tuple[float, float]] = []
    for wp in waypoints:
        x_m = -(wp.x - ox) / 1000.0
        y_m = (wp.y - oy) / 1000.0
        out.append((x_m, y_m))
    return out


def starting_heading_rad(pts: list[tuple[float, float]]) -> float:
    """Heading of first segment, snapped to 90° (wall-aligned placement)."""
    if len(pts) < 2:
        return math.pi / 2.0
    dx = pts[1][0] - pts[0][0]
    dy = pts[1][1] - pts[0][1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return math.pi / 2.0
    raw = math.atan2(dy, dx)
    quarter = round(raw / (math.pi / 2.0))
    snapped = quarter * (math.pi / 2.0)
    while snapped > math.pi:
        snapped -= 2.0 * math.pi
    while snapped <= -math.pi:
        snapped += 2.0 * math.pi
    return snapped


def _header_text(
    pts: list[tuple[float, float]],
    heading: float,
    *,
    cylinder_seg_lo: int = -1,
    cylinder_seg_hi: int = -1,
    front_wall_segs: list[int] | None = None,
) -> str:
    segs = sorted({int(s) for s in (front_wall_segs or []) if int(s) >= 0})
    lines = [
        "#pragma once",
        "",
        f"#define INITIAL_HEADING_RAD ({_fmt(heading)})",
        # PurePursuit segment indices; -1 = no cylinder slow zone (wall maze).
        f"#define CYLINDER_SEG_LO ({int(cylinder_seg_lo)})",
        f"#define CYLINDER_SEG_HI ({int(cylinder_seg_hi)})",
        "",
        # Segments where front ToF may correct pose (path approaches a wall face).
        "const uint8_t FRONT_WALL_SEGS[] = {",
    ]
    if segs:
        lines.append("  " + ", ".join(str(s) for s in segs) + ",")
    lines.extend(
        [
            "};",
            "const size_t FRONT_WALL_SEG_COUNT = "
            "sizeof(FRONT_WALL_SEGS) / sizeof(FRONT_WALL_SEGS[0]);",
            "",
            "const Waypoint2D WAYPOINTS[] = {",
        ]
    )
    for x, y in pts:
        lines.append(f"  {{{_fmt(x)}, {_fmt(y)}}},")
    lines.extend(
        [
            "};",
            "const size_t WAYPOINT_LEN = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);",
            "",
        ]
    )
    return "\n".join(lines)


def _unit_edge_present(
    h_bits: set[int],
    v_bits: set[int],
    *,
    kind: str,
    a: int,
    b: int,
    h_stride: int,
    v_stride: int,
) -> bool:
    if kind == "h":
        # a = pole row [0, v_stride], b = west pole col [0, h_stride)
        if not (0 <= a <= v_stride and 0 <= b < h_stride):
            return False
        return (a * h_stride + b) in h_bits
    # a = pole col [0, h_stride], b = north pole row [0, v_stride)
    if not (0 <= a <= h_stride and 0 <= b < v_stride):
        return False
    return (a * v_stride + b) in v_bits


def front_wall_seg_indices(
    waypoints: list[Waypoint],
    walls: list[WallSeg],
    cell_mm: float,
    *,
    pole_cols: int,
    pole_rows: int,
    cylinder_seg_lo: int = -1,
    cylinder_seg_hi: int = -1,
    max_approach_mm: float = 200.0,
) -> list[int]:
    """PurePursuit segment indices that approach a grid wall face.

    For edge i → i+1, if travel is axis-aligned and the far face of the end
    cell has a wall within ``max_approach_mm`` of the segment end, mark i.
    Cylinder slow-zone segments are excluded.
    """
    if len(waypoints) < 2:
        return []

    h_bits, v_bits = wall_edge_indices(walls, pole_cols, pole_rows)
    h_stride = pole_cols - 1
    v_stride = pole_rows - 1
    eps = 1e-3
    marked: list[int] = []

    for i in range(len(waypoints) - 1):
        if (
            cylinder_seg_lo >= 0
            and cylinder_seg_hi >= 0
            and cylinder_seg_lo <= i <= cylinder_seg_hi
        ):
            continue

        a = waypoints[i]
        b = waypoints[i + 1]
        dx = b.x - a.x
        dy = b.y - a.y
        if abs(dx) < eps and abs(dy) < eps:
            continue

        col = int(math.floor(b.x / cell_mm))
        row = int(math.floor(b.y / cell_mm))
        if col < 0 or row < 0 or col >= h_stride or row >= v_stride:
            continue

        # Maze: x right, y down.
        if abs(dx) >= abs(dy):
            if dx > 0:
                # +x → east face of end cell
                has = _unit_edge_present(
                    h_bits, v_bits, kind="v", a=col + 1, b=row,
                    h_stride=h_stride, v_stride=v_stride,
                )
                face_x = (col + 1) * cell_mm
                dist = abs(face_x - b.x)
            else:
                has = _unit_edge_present(
                    h_bits, v_bits, kind="v", a=col, b=row,
                    h_stride=h_stride, v_stride=v_stride,
                )
                face_x = col * cell_mm
                dist = abs(b.x - face_x)
        else:
            if dy > 0:
                # +y → south face
                has = _unit_edge_present(
                    h_bits, v_bits, kind="h", a=row + 1, b=col,
                    h_stride=h_stride, v_stride=v_stride,
                )
                face_y = (row + 1) * cell_mm
                dist = abs(face_y - b.y)
            else:
                has = _unit_edge_present(
                    h_bits, v_bits, kind="h", a=row, b=col,
                    h_stride=h_stride, v_stride=v_stride,
                )
                face_y = row * cell_mm
                dist = abs(b.y - face_y)

        if has and dist <= max_approach_mm + 1e-6:
            marked.append(i)

    return marked


def cylinder_seg_range(
    waypoints: list[Waypoint],
    region_cells: set[tuple[int, int]] | list[tuple[int, int]] | None,
    cell_mm: float,
) -> tuple[int, int]:
    """Waypoint/segment index span that lies in the cylinder cell block.

    PurePursuit ``segment_`` indexes the start of the current edge, so these
    values plug straight into ``CYLINDER_SEG_LO`` / ``HI``. Returns ``(-1, -1)``
    when there is no region or no in-block waypoints.
    """
    if not region_cells or len(waypoints) < 2:
        return -1, -1
    cells = [(int(c), int(r)) for c, r in region_cells]
    col0 = min(c for c, _ in cells)
    col1 = max(c for c, _ in cells)
    row0 = min(r for _, r in cells)
    row1 = max(r for _, r in cells)
    x0 = col0 * cell_mm
    x1 = (col1 + 1) * cell_mm
    y0 = row0 * cell_mm
    y1 = (row1 + 1) * cell_mm
    inside = [
        i
        for i, wp in enumerate(waypoints)
        if x0 <= wp.x <= x1 and y0 <= wp.y <= y1
    ]
    if not inside:
        return -1, -1
    return int(inside[0]), int(inside[-1])


def export_waypoints_snippet(
    planned: PlannedPath,
    params: Params,
    output_dir: str | Path,
    *,
    prefix: str = "10_waypoints",
    sketch_header: str | Path | None = None,
    region_cells: set[tuple[int, int]] | list[tuple[int, int]] | None = None,
    walls: WallDetection | list[WallSeg] | None = None,
) -> Path:
    """Write includeable header for the Arduino sketch (+ copy under outputs/)."""
    maze = params.maze
    path_cfg = params.path
    cell = float(maze["cell_size_mm"])
    start_c, start_r = int(path_cfg["start_cell"][0]), int(path_cfg["start_cell"][1])
    origin = _cell_center_mm(start_c, start_r, cell)
    pts = waypoints_robot_m(planned.waypoints, origin)
    heading = starting_heading_rad(pts)

    pole_cols = int(maze["pole_grid_cols"])
    pole_rows = int(maze["pole_grid_rows"])

    cyl_lo, cyl_hi = -1, -1
    if region_cells and str(planned.mode).startswith("cylinders"):
        cyl_lo, cyl_hi = cylinder_seg_range(
            planned.waypoints, region_cells, cell
        )

    wall_list: list[WallSeg] = []
    if isinstance(walls, WallDetection):
        wall_list = list(walls.walls)
    elif walls is not None:
        wall_list = list(walls)

    front_segs = front_wall_seg_indices(
        planned.waypoints,
        wall_list,
        cell,
        pole_cols=pole_cols,
        pole_rows=pole_rows,
        cylinder_seg_lo=cyl_lo,
        cylinder_seg_hi=cyl_hi,
        max_approach_mm=200.0,
    )

    text = _header_text(
        pts,
        heading,
        cylinder_seg_lo=cyl_lo,
        cylinder_seg_hi=cyl_hi,
        front_wall_segs=front_segs,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    debug_path = out / f"{prefix}.hpp"
    debug_path.write_text(text, encoding="utf-8")

    csv_path = out / f"{prefix}.csv"
    csv_path.write_text(
        f"# initial_heading_rad={heading:.6f}\n"
        f"# cylinder_seg_lo={cyl_lo} cylinder_seg_hi={cyl_hi}\n"
        f"# front_wall_segs={','.join(str(s) for s in front_segs)}\n"
        "i,x_m,y_m\n"
        + "\n".join(f"{i},{x:.4f},{y:.4f}" for i, (x, y) in enumerate(pts))
        + "\n",
        encoding="utf-8",
    )

    if sketch_header is None:
        sketch_header = params.output.get("waypoints_header", "../waypoints_planned.hpp")
    header_path = Path(sketch_header)
    if not header_path.is_absolute():
        header_path = (out.parent / header_path).resolve()
    header_path.parent.mkdir(parents=True, exist_ok=True)
    header_path.write_text(text, encoding="utf-8")
    if cyl_lo >= 0:
        print(f"  cylinder segments = [{cyl_lo}, {cyl_hi}] (slow zone)")
    else:
        print("  cylinder segments = none")
    if front_segs:
        print(f"  front-wall segs = {front_segs}")
    else:
        print("  front-wall segs = none")
    return header_path


def _to_robot_xy(x_mm: float, y_mm: float, origin_mm: tuple[float, float]) -> tuple[float, float]:
    ox, oy = origin_mm
    return (-(x_mm - ox) / 1000.0, (y_mm - oy) / 1000.0)


def _line_hpp(name: str, segs: list[tuple[float, float, float, float]]) -> list[str]:
    lines = [f"const LineSeg2D {name}[] PROGMEM = {{"]
    for x1, y1, x2, y2 in segs:
        lines.append(f"  {{{_fmt(x1)}, {_fmt(y1)}, {_fmt(x2)}, {_fmt(y2)}}},")
    lines.extend(
        [
            "};",
            f"const size_t {name}_LEN = sizeof({name}) / sizeof({name}[0]);",
            "",
        ]
    )
    return lines


def _pack_bits(nbits: int, set_indices: set[int]) -> list[int]:
    nbytes = (nbits + 7) // 8
    buf = [0] * nbytes
    for i in set_indices:
        if 0 <= i < nbits:
            buf[i >> 3] |= 1 << (i & 7)
    return buf


def _bytes_hpp(name: str, data: list[int]) -> list[str]:
    lines = [f"const uint8_t {name}[] PROGMEM = {{"]
    row: list[str] = []
    for i, b in enumerate(data):
        row.append(f"0x{b:02X}u")
        if len(row) == 12 or i == len(data) - 1:
            lines.append("  " + ", ".join(row) + (", " if i < len(data) - 1 else ""))
            row = []
    lines.extend(
        [
            "};",
            f"const size_t {name}_LEN = sizeof({name}) / sizeof({name}[0]);",
            "",
        ]
    )
    return lines


def wall_edge_indices(
    walls: list[WallSeg],
    pole_cols: int,
    pole_rows: int,
) -> tuple[set[int], set[int]]:
    """Expand wall segs into unit-edge bits.

    H bit: row * (pole_cols-1) + col   (edge between col..col+1 at pole-row)
    V bit: col * (pole_rows-1) + row   (edge between row..row+1 at pole-col)
    """
    h_stride = pole_cols - 1
    v_stride = pole_rows - 1
    h_bits: set[int] = set()
    v_bits: set[int] = set()
    for w in walls:
        if w.b1 <= w.b0:
            continue
        if w.kind == "h":
            if not (0 <= w.a < pole_rows):
                continue
            for c in range(w.b0, w.b1):
                if 0 <= c < h_stride:
                    h_bits.add(w.a * h_stride + c)
        else:
            if not (0 <= w.a < pole_cols):
                continue
            for r in range(w.b0, w.b1):
                if 0 <= r < v_stride:
                    v_bits.add(w.a * v_stride + r)
    return h_bits, v_bits


def pole_bit_indices(
    poles_ij: set[tuple[int, int]],
    pole_cols: int,
    pole_rows: int,
) -> set[int]:
    bits: set[int] = set()
    for i, j in poles_ij:
        if 0 <= i < pole_cols and 0 <= j < pole_rows:
            bits.add(j * pole_cols + i)
    return bits


def build_walls_header_text(
    *,
    walls: list[WallSeg],
    poles_ij: set[tuple[int, int]],
    legacy_faces_robot: list[tuple[float, float, float, float]],
    cell_mm: float,
    wall_th_mm: float,
    post_mm: float,
    pole_cols: int,
    pole_rows: int,
    origin_mm: tuple[float, float],
) -> str:
    h_bits, v_bits = wall_edge_indices(walls, pole_cols, pole_rows)
    p_bits = pole_bit_indices(poles_ij, pole_cols, pole_rows)
    n_h = pole_rows * (pole_cols - 1)
    n_v = pole_cols * (pole_rows - 1)
    n_p = pole_cols * pole_rows

    ox_m = origin_mm[0] / 1000.0
    oy_m = origin_mm[1] / 1000.0

    lines = [
        "#pragma once",
        "",
        "#include <Arduino.h>",
        "",
        "// Orthogonal maze: wall/pole presence bitsets + fixed geometry.",
        "// Maze frame: x right, y down, origin at pole (0,0). Robot origin = start cell centre.",
        "// H index = poleRow * MAP_H_STRIDE + westPoleCol",
        "// V index = poleCol * MAP_V_STRIDE + northPoleRow",
        "// Pole index = poleRow * MAP_POLE_COLS + poleCol",
        "// MAP_LEGACY_FACES: non-grid obstacles in robot frame (outer shell, diagonals, extras).",
        f"static constexpr float MAP_CELL_M = {_fmt(cell_mm / 1000.0)};",
        f"static constexpr float MAP_WALL_TH_M = {_fmt(wall_th_mm / 1000.0)};",
        f"static constexpr float MAP_POST_M = {_fmt(post_mm / 1000.0)};",
        f"static constexpr uint8_t MAP_POLE_COLS = {pole_cols};",
        f"static constexpr uint8_t MAP_POLE_ROWS = {pole_rows};",
        f"static constexpr uint8_t MAP_H_STRIDE = {pole_cols - 1};",
        f"static constexpr uint8_t MAP_V_STRIDE = {pole_rows - 1};",
        f"static constexpr uint16_t MAP_H_BITS = {n_h};",
        f"static constexpr uint16_t MAP_V_BITS = {n_v};",
        f"static constexpr uint16_t MAP_POLE_BITS = {n_p};",
        f"static constexpr float MAP_ORIGIN_X_M = {_fmt(ox_m)};",
        f"static constexpr float MAP_ORIGIN_Y_M = {_fmt(oy_m)};",
        "",
        "struct LineSeg2D { float x1; float y1; float x2; float y2; };",
        "",
    ]
    lines.extend(_bytes_hpp("MAP_H_WALLS", _pack_bits(n_h, h_bits)))
    lines.extend(_bytes_hpp("MAP_V_WALLS", _pack_bits(n_v, v_bits)))
    lines.extend(_bytes_hpp("MAP_POLES", _pack_bits(n_p, p_bits)))
    lines.extend(_line_hpp("MAP_LEGACY_FACES", legacy_faces_robot))
    return "\n".join(lines)


def _legacy_extras_mm(params: Params) -> list[tuple[float, float, float, float]]:
    """Optional non-grid segments in maze mm from param map.legacy_segments_mm."""
    segs = params.map.get("legacy_segments_mm", []) or []
    out: list[tuple[float, float, float, float]] = []
    for s in segs:
        if len(s) >= 4:
            out.append((float(s[0]), float(s[1]), float(s[2]), float(s[3])))
    return out


def export_walls_header(
    walls: WallDetection,
    shell: ShellResult,
    fine: FineWarpResult,
    params: Params,
    output_dir: str | Path,
    *,
    prefix: str = "11_walls",
    sketch_header: str | Path | None = None,
) -> Path:
    maze = params.maze
    path_cfg = params.path
    cell_mm = float(maze["cell_size_mm"])
    wall_th_mm = float(maze["wall_thickness_mm"])
    post_mm = float(maze["post_size_mm"])
    pole_cols = int(maze["pole_grid_cols"])
    pole_rows = int(maze["pole_grid_rows"])
    start_c = int(path_cfg["start_cell"][0])
    start_r = int(path_cfg["start_cell"][1])
    origin_mm = _cell_center_mm(start_c, start_r, cell_mm)

    poles = _pole_map(fine, params)
    poles_ij = set(poles.keys())

    legacy_mm: list[tuple[float, float, float, float]] = []
    for seg in shell.segments:
        i0 = seg.inner_mm[0]
        i1 = seg.inner_mm[1]
        o0 = seg.outer_mm[0]
        o1 = seg.outer_mm[1]
        legacy_mm.append((float(i0[0]), float(i0[1]), float(i1[0]), float(i1[1])))
        legacy_mm.append((float(o0[0]), float(o0[1]), float(o1[0]), float(o1[1])))
    legacy_mm.extend(_legacy_extras_mm(params))

    legacy_robot = [
        (*_to_robot_xy(x1, y1, origin_mm), *_to_robot_xy(x2, y2, origin_mm))
        for x1, y1, x2, y2 in legacy_mm
    ]

    text = build_walls_header_text(
        walls=walls.walls,
        poles_ij=poles_ij,
        legacy_faces_robot=legacy_robot,
        cell_mm=cell_mm,
        wall_th_mm=wall_th_mm,
        post_mm=post_mm,
        pole_cols=pole_cols,
        pole_rows=pole_rows,
        origin_mm=origin_mm,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    debug_path = out / f"{prefix}.hpp"
    debug_path.write_text(text, encoding="utf-8")

    if sketch_header is None:
        sketch_header = params.output.get("walls_header", "../walls_planned.hpp")
    header_path = Path(sketch_header)
    if not header_path.is_absolute():
        header_path = (out.parent / header_path).resolve()
    header_path.parent.mkdir(parents=True, exist_ok=True)
    header_path.write_text(text, encoding="utf-8")
    return header_path


def select_export_path(
    centers: PlannedPath,
    minturns: PlannedPath,
    fastest: PlannedPath,
    fastest_minturns: PlannedPath,
    params: Params,
    *,
    has_cylinders: bool = False,
    cylinder_paths: dict[str, PlannedPath] | None = None,
    cylinders_path: PlannedPath | None = None,
) -> PlannedPath:
    """Pick the path written to the robot waypoint header.

    Wall mazes use ``planning.export_mode`` (default minturns / orthogonal).
    When cylinders were detected, use ``planning.cylinder_export_mode``
    (``cylinders1``…``cylinders7`` = hybrid at 50/40/30/20/10/5/0 mm pad;
    ``cylinders8`` = minturns through cylinder section when possible, else hybrid).
    """
    plan = params.planning
    cyl_paths = dict(cylinder_paths or {})
    if cylinders_path is not None and cylinders_path.mode:
        cyl_paths.setdefault(cylinders_path.mode, cylinders_path)

    if has_cylinders:
        mode = str(
            plan.get("cylinder_export_mode", "cylinders1")
        ).lower().strip()
    else:
        mode = str(plan.get("export_mode", "minturns")).lower().strip()

    # Alias: cylinders / hybrid → cylinders1 (50 mm)
    if mode in ("cylinders", "path_cylinders", "hybrid", "cylinder"):
        mode = "cylinders1"

    if mode.startswith("cylinders"):
        picked = cyl_paths.get(mode)
        if picked is not None and picked.waypoints:
            return picked
        # Fall back to any successful named pad (largest pad first if sorted).
        for name in sorted(
            cyl_paths.keys(),
            key=lambda n: (
                -float(
                    (plan.get("cylinder_path_pads_mm") or {}).get(n, 0)
                )
                if isinstance(plan.get("cylinder_path_pads_mm"), dict)
                else 0
            ),
        ):
            p = cyl_paths[name]
            if p.waypoints:
                return p
        return fastest
    if mode in ("centers", "cell_centers", "orthogonal"):
        return centers
    if mode in ("fastest", "shortest"):
        return fastest
    if mode in ("fastest_minturns", "fast_minturns", "straight", "straights"):
        return fastest_minturns
    if mode in ("minturns", "cell_centers_minturns", "min_turns"):
        return minturns
    return minturns
