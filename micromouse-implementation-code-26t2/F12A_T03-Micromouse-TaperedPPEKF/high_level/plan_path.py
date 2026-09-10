from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from occupancy import OccupancyMap, _mm_to_cell
from poles import Params


@dataclass
class Waypoint:
    x: float  # mm, origin at pole (0,0)
    y: float


@dataclass
class PlannedPath:
    mode: str
    waypoints: list[Waypoint]
    path_cells: list[tuple[int, int]]  # occupancy grid cells along path (for debug)
    length_mm: float


def load_planned_path(path: str | Path) -> PlannedPath:
    """Read a ``09_path_*.yaml`` written by ``save_path_debug``."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    waypoints = [
        Waypoint(x=float(x), y=float(y))
        for x, y in (data.get("waypoints_mm") or [])
    ]
    return PlannedPath(
        mode=str(data.get("mode") or Path(path).stem),
        waypoints=waypoints,
        path_cells=[],
        length_mm=float(data.get("length_mm") or 0.0),
    )


def _in_bounds(grid: np.ndarray, x: int, y: int) -> bool:
    h, w = grid.shape
    return 0 <= x < w and 0 <= y < h


def _neighbors(x: int, y: int, allow_diagonal: bool) -> list[tuple[int, int]]:
    steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if allow_diagonal:
        steps.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])
    return [(x + dx, y + dy) for dx, dy in steps]


def _move_cost(dx: int, dy: int) -> float:
    return math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0


def _heuristic(x: int, y: int, gx: int, gy: int, allow_diagonal: bool) -> float:
    dx, dy = abs(gx - x), abs(gy - y)
    return math.hypot(dx, dy) if allow_diagonal else float(dx + dy)


def _corner_clear(grid: np.ndarray, x: int, y: int, nx: int, ny: int) -> bool:
    if x == nx or y == ny:
        return True
    return grid[y, nx] == 0 and grid[ny, x] == 0


def astar(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    allow_diagonal: bool,
) -> list[tuple[int, int]]:
    if not _in_bounds(grid, *start) or not _in_bounds(grid, *goal):
        raise ValueError("Start or goal out of bounds")
    if grid[start[1], start[0]] != 0 or grid[goal[1], goal[0]] != 0:
        raise ValueError("Start or goal is inside an obstacle")

    open_set: list[tuple[float, int, int]] = []
    heapq.heappush(open_set, (0.0, start[0], start[1]))
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    g_score = {start: 0.0}

    while open_set:
        _, x, y = heapq.heappop(open_set)
        if (x, y) == goal:
            break
        for nx, ny in _neighbors(x, y, allow_diagonal):
            if not _in_bounds(grid, nx, ny) or grid[ny, nx] != 0:
                continue
            if not _corner_clear(grid, x, y, nx, ny):
                continue
            tentative = g_score[(x, y)] + _move_cost(nx - x, ny - y)
            if tentative < g_score.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = (x, y)
                g_score[(nx, ny)] = tentative
                f = tentative + _heuristic(nx, ny, goal[0], goal[1], allow_diagonal)
                heapq.heappush(open_set, (f, nx, ny))
    else:
        raise RuntimeError("No path found")

    path: list[tuple[int, int]] = []
    node: tuple[int, int] | None = goal
    while node is not None:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    return path


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points


def line_free(grid: np.ndarray, p0: tuple[int, int], p1: tuple[int, int]) -> bool:
    for x, y in bresenham(p0[0], p0[1], p1[0], p1[1]):
        if not _in_bounds(grid, x, y) or grid[y, x] != 0:
            return False
    return True


def los_shortcut(
    path: list[tuple[int, int]],
    grid: np.ndarray,
    *,
    orthogonal_only: bool,
) -> list[tuple[int, int]]:
    if len(path) < 2:
        return path
    if not orthogonal_only:
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not line_free(grid, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        best_j = i + 1
        best_corner: tuple[int, int] | None = None
        for j in range(i + 1, len(path)):
            pi, pj = path[i], path[j]
            if (pi[1] == pj[1] or pi[0] == pj[0]) and line_free(grid, pi, pj):
                best_j = j
                best_corner = None
                continue
            c1 = (pj[0], pi[1])
            c2 = (pi[0], pj[1])
            if (
                _in_bounds(grid, *c1)
                and grid[c1[1], c1[0]] == 0
                and line_free(grid, pi, c1)
                and line_free(grid, c1, pj)
            ):
                best_j = j
                best_corner = c1
            elif (
                _in_bounds(grid, *c2)
                and grid[c2[1], c2[0]] == 0
                and line_free(grid, pi, c2)
                and line_free(grid, c2, pj)
            ):
                best_j = j
                best_corner = c2
        if best_corner is not None:
            out.append(best_corner)
        out.append(path[best_j])
        i = best_j
    return out


def rdp(points: list[tuple[float, float]], epsilon: float) -> list[tuple[float, float]]:
    if len(points) < 3:
        return points
    start = np.array(points[0], dtype=np.float64)
    end = np.array(points[-1], dtype=np.float64)
    line = end - start
    line_len = float(np.linalg.norm(line))
    if line_len < 1e-9:
        return [points[0], points[-1]]
    dists = []
    for p in points[1:-1]:
        vec = np.array(p, dtype=np.float64) - start
        proj = float(np.dot(vec, line) / line_len)
        closest = start + (line / line_len) * proj
        dists.append(float(np.linalg.norm(np.array(p, dtype=np.float64) - closest)))
    idx = int(np.argmax(dists)) + 1
    if dists[idx - 1] > epsilon:
        left = rdp(points[: idx + 1], epsilon)
        right = rdp(points[idx:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def _path_length_mm(wps: list[Waypoint]) -> float:
    total = 0.0
    for a, b in zip(wps, wps[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def _mm_to_occ(occ: OccupancyMap, x_mm: float, y_mm: float) -> tuple[int, int]:
    return _mm_to_cell(x_mm, y_mm, occ.origin_mm, occ.cell_mm)


def _occ_to_mm(occ: OccupancyMap, x: int, y: int) -> tuple[float, float]:
    return (
        occ.origin_mm[0] + (x + 0.5) * occ.cell_mm,
        occ.origin_mm[1] + (y + 0.5) * occ.cell_mm,
    )


def _cell_center_mm(col: int, row: int, cell_size_mm: float) -> tuple[float, float]:
    return ((col + 0.5) * cell_size_mm, (row + 0.5) * cell_size_mm)


def _free_at_mm(occ: OccupancyMap, x_mm: float, y_mm: float, *, inflated: bool = True) -> bool:
    grid = occ.inflated if inflated else occ.occupancy
    gx, gy = _mm_to_occ(occ, x_mm, y_mm)
    return _in_bounds(grid, gx, gy) and grid[gy, gx] == 0


def _edge_free(
    occ: OccupancyMap,
    a: tuple[float, float],
    b: tuple[float, float],
) -> bool:
    p0 = _mm_to_occ(occ, a[0], a[1])
    p1 = _mm_to_occ(occ, b[0], b[1])
    return line_free(occ.inflated, p0, p1)


def _maze_centres(
    occ: OccupancyMap, params: Params
) -> tuple[dict[tuple[int, int], tuple[float, float]], tuple[int, int], tuple[int, int]]:
    maze = params.maze
    path_cfg = params.path
    cols = int(maze["grid_cols"])
    rows = int(maze["grid_rows"])
    cell = float(maze["cell_size_mm"])
    start = (int(path_cfg["start_cell"][0]), int(path_cfg["start_cell"][1]))
    goal = (int(path_cfg["goal_cell"][0]), int(path_cfg["goal_cell"][1]))
    centres = {
        (c, r): _cell_center_mm(c, r, cell)
        for r in range(rows)
        for c in range(cols)
        if _free_at_mm(occ, *_cell_center_mm(c, r, cell))
    }
    if start not in centres:
        raise ValueError(f"start cell {start} centre is blocked")
    if goal not in centres:
        raise ValueError(f"goal cell {goal} centre is blocked")
    return centres, start, goal


def _cells_to_planned(
    mode: str,
    cells: list[tuple[int, int]],
    centres: dict[tuple[int, int], tuple[float, float]],
    occ: OccupancyMap,
) -> PlannedPath:
    waypoints = [Waypoint(x=centres[c][0], y=centres[c][1]) for c in cells]
    occ_cells = [_mm_to_occ(occ, wp.x, wp.y) for wp in waypoints]
    return PlannedPath(
        mode=mode,
        waypoints=waypoints,
        path_cells=occ_cells,
        length_mm=_path_length_mm(waypoints),
    )


def plan_manual_cells(occ: OccupancyMap, params: Params) -> PlannedPath | None:
    """Build a path from path.manual_cells ([col, row] list). None if unset/empty."""
    raw = params.path.get("manual_cells") or []
    if not raw:
        return None
    cell = float(params.maze["cell_size_mm"])
    cells = [(int(item[0]), int(item[1])) for item in raw]
    if len(cells) < 2:
        raise ValueError("path.manual_cells needs at least two [col, row] entries")
    centres = {(c, r): _cell_center_mm(c, r, cell) for c, r in cells}
    return _cells_to_planned("manual", cells, centres, occ)


def plan_cell_centers(occ: OccupancyMap, params: Params) -> PlannedPath:
    """Orthogonal A* on maze cells; waypoint at every cell centre on the route."""
    centres, start, goal = _maze_centres(occ, params)
    start_c, start_r = start
    goal_c, goal_r = goal

    open_set: list[tuple[float, int, int]] = []
    heapq.heappush(open_set, (0.0, start_c, start_r))
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    g_score = {start: 0.0}

    while open_set:
        _, c, r = heapq.heappop(open_set)
        if (c, r) == goal:
            break
        for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nc, nr = c + dc, r + dr
            if (nc, nr) not in centres:
                continue
            if not _edge_free(occ, centres[(c, r)], centres[(nc, nr)]):
                continue
            tentative = g_score[(c, r)] + 1.0
            if tentative < g_score.get((nc, nr), float("inf")):
                came_from[(nc, nr)] = (c, r)
                g_score[(nc, nr)] = tentative
                f = tentative + abs(goal_c - nc) + abs(goal_r - nr)
                heapq.heappush(open_set, (f, nc, nr))
    else:
        raise RuntimeError("No cell-centre path found")

    cells: list[tuple[int, int]] = []
    node: tuple[int, int] | None = goal
    while node is not None:
        cells.append(node)
        node = came_from.get(node)
    cells.reverse()
    return _cells_to_planned("cell_centers", cells, centres, occ)


def plan_cell_centers_minturns(occ: OccupancyMap, params: Params) -> PlannedPath:
    """Same as cell_centers, but penalise direction changes (fewer turns).

    Step cost = 1 per cell; each 90° turn adds planning.turn_penalty (default 0.1)
    so equal-length routes prefer fewer corners without taking longer detours.
    """
    centres, start, goal = _maze_centres(occ, params)
    start_c, start_r = start
    goal_c, goal_r = goal
    turn_penalty = float(params.planning.get("turn_penalty", 0.1))

    # State: (col, row, dir) with dir in {0:W,1:E,2:N,3:S}; -1 = start (no heading)
    dirs = ((-1, 0), (1, 0), (0, -1), (0, 1))
    start_state = (start_c, start_r, -1)
    open_set: list[tuple[float, int, int, int]] = []
    heapq.heappush(open_set, (0.0, start_c, start_r, -1))
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {
        start_state: None
    }
    g_score: dict[tuple[int, int, int], float] = {start_state: 0.0}
    closed: set[tuple[int, int, int]] = set()
    best_goal: tuple[int, int, int] | None = None

    while open_set:
        _f, c, r, d = heapq.heappop(open_set)
        state = (c, r, d)
        if state in closed:
            continue
        closed.add(state)
        if (c, r) == goal:
            best_goal = state
            break

        g = g_score[state]
        for nd, (dc, dr) in enumerate(dirs):
            nc, nr = c + dc, r + dr
            if (nc, nr) not in centres:
                continue
            if not _edge_free(occ, centres[(c, r)], centres[(nc, nr)]):
                continue
            step = 1.0 + (turn_penalty if d >= 0 and nd != d else 0.0)
            tentative = g + step
            nstate = (nc, nr, nd)
            if nstate in closed:
                continue
            if tentative < g_score.get(nstate, float("inf")):
                came_from[nstate] = state
                g_score[nstate] = tentative
                h = float(abs(goal_c - nc) + abs(goal_r - nr))
                heapq.heappush(open_set, (tentative + h, nc, nr, nd))

    if best_goal is None:
        raise RuntimeError("No min-turn cell-centre path found")

    cells_rev: list[tuple[int, int]] = []
    node: tuple[int, int, int] | None = best_goal
    while node is not None:
        cells_rev.append((node[0], node[1]))
        node = came_from.get(node)
    cells_rev.reverse()
    cells: list[tuple[int, int]] = []
    for cr in cells_rev:
        if not cells or cells[-1] != cr:
            cells.append(cr)
    return _cells_to_planned("cell_centers_minturns", cells, centres, occ)


def plan_fastest(occ: OccupancyMap, params: Params) -> PlannedPath:
    """Fine-grid A* (8-connected) + LOS shortcut + RDP — shortest feasible path."""
    maze = params.maze
    path_cfg = params.path
    plan_cfg = params.planning
    cell = float(maze["cell_size_mm"])
    start_c, start_r = [int(v) for v in path_cfg["start_cell"]]
    goal_c, goal_r = [int(v) for v in path_cfg["goal_cell"]]
    start_mm = _cell_center_mm(start_c, start_r, cell)
    goal_mm = _cell_center_mm(goal_c, goal_r, cell)
    start = _mm_to_occ(occ, *start_mm)
    goal = _mm_to_occ(occ, *goal_mm)

    raw = astar(occ.inflated, start, goal, allow_diagonal=True)
    shortcut = los_shortcut(raw, occ.inflated, orthogonal_only=False)
    metric = [_occ_to_mm(occ, x, y) for x, y in shortcut]
    simplified = rdp(metric, float(plan_cfg.get("rdp_epsilon_mm", 8.0)))
    waypoints = [Waypoint(x=p[0], y=p[1]) for p in simplified]
    return PlannedPath(
        mode="fastest",
        waypoints=waypoints,
        path_cells=shortcut,
        length_mm=_path_length_mm(waypoints),
    )


def _turn_steps(d_from: int, d_to: int) -> int:
    """Smallest number of 45° steps between two 8-connected headings (0..7)."""
    if d_from < 0:
        return 0
    return min((d_to - d_from) % 8, (d_from - d_to) % 8)


def astar_minturns(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    turn_penalty: float,
) -> list[tuple[int, int]]:
    """8-connected A* with heading state; penalise direction changes."""
    if not _in_bounds(grid, *start) or not _in_bounds(grid, *goal):
        raise ValueError("Start or goal out of bounds")
    if grid[start[1], start[0]] != 0 or grid[goal[1], goal[0]] != 0:
        raise ValueError("Start or goal is inside an obstacle")

    # Headings in 45° steps (dx, dy) around the circle
    dirs = (
        (1, 0),
        (1, 1),
        (0, 1),
        (-1, 1),
        (-1, 0),
        (-1, -1),
        (0, -1),
        (1, -1),
    )
    start_state = (start[0], start[1], -1)
    open_set: list[tuple[float, int, int, int]] = []
    heapq.heappush(open_set, (0.0, start[0], start[1], -1))
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {
        start_state: None
    }
    g_score: dict[tuple[int, int, int], float] = {start_state: 0.0}
    closed: set[tuple[int, int, int]] = set()
    best_goal: tuple[int, int, int] | None = None

    while open_set:
        _f, x, y, d = heapq.heappop(open_set)
        state = (x, y, d)
        if state in closed:
            continue
        closed.add(state)
        if (x, y) == goal:
            best_goal = state
            break

        g = g_score[state]
        for nd, (dx, dy) in enumerate(dirs):
            nx, ny = x + dx, y + dy
            if not _in_bounds(grid, nx, ny) or grid[ny, nx] != 0:
                continue
            if not _corner_clear(grid, x, y, nx, ny):
                continue
            step = _move_cost(dx, dy) + turn_penalty * float(_turn_steps(d, nd))
            tentative = g + step
            nstate = (nx, ny, nd)
            if nstate in closed:
                continue
            if tentative < g_score.get(nstate, float("inf")):
                came_from[nstate] = state
                g_score[nstate] = tentative
                h = math.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(open_set, (tentative + h, nx, ny, nd))

    if best_goal is None:
        raise RuntimeError("No min-turn fine-grid path found")

    path: list[tuple[int, int]] = []
    node: tuple[int, int, int] | None = best_goal
    while node is not None:
        path.append((node[0], node[1]))
        node = came_from.get(node)
    path.reverse()
    # Drop consecutive duplicates (start state shares cell with first step)
    out: list[tuple[int, int]] = []
    for p in path:
        if not out or out[-1] != p:
            out.append(p)
    return out


def plan_fastest_minturns(occ: OccupancyMap, params: Params) -> PlannedPath:
    """Fine-grid 8-connected A* with turn penalty + LOS + RDP — short & straight."""
    maze = params.maze
    path_cfg = params.path
    plan_cfg = params.planning
    cell = float(maze["cell_size_mm"])
    start_c, start_r = [int(v) for v in path_cfg["start_cell"]]
    goal_c, goal_r = [int(v) for v in path_cfg["goal_cell"]]
    start_mm = _cell_center_mm(start_c, start_r, cell)
    goal_mm = _cell_center_mm(goal_c, goal_r, cell)
    start = _mm_to_occ(occ, *start_mm)
    goal = _mm_to_occ(occ, *goal_mm)

    turn_penalty = float(
        plan_cfg.get("fastest_turn_penalty", plan_cfg.get("turn_penalty", 1.0))
    )
    raw = astar_minturns(
        occ.inflated, start, goal, turn_penalty=turn_penalty
    )
    shortcut = los_shortcut(raw, occ.inflated, orthogonal_only=False)
    metric = [_occ_to_mm(occ, x, y) for x, y in shortcut]
    simplified = rdp(metric, float(plan_cfg.get("rdp_epsilon_mm", 8.0)))
    waypoints = [Waypoint(x=p[0], y=p[1]) for p in simplified]
    return PlannedPath(
        mode="fastest_minturns",
        waypoints=waypoints,
        path_cells=shortcut,
        length_mm=_path_length_mm(waypoints),
    )


def _fine_segment_mm(
    occ: OccupancyMap,
    params: Params,
    start_mm: tuple[float, float],
    goal_mm: tuple[float, float],
) -> tuple[list[Waypoint], list[tuple[int, int]]]:
    """Fastest-style fine path between two maze-mm points."""
    start = _mm_to_occ(occ, *start_mm)
    goal = _mm_to_occ(occ, *goal_mm)
    raw = astar(occ.inflated, start, goal, allow_diagonal=True)
    shortcut = los_shortcut(raw, occ.inflated, orthogonal_only=False)
    metric = [_occ_to_mm(occ, x, y) for x, y in shortcut]
    simplified = rdp(metric, float(params.planning.get("rdp_epsilon_mm", 8.0)))
    waypoints = [Waypoint(x=p[0], y=p[1]) for p in simplified]
    return waypoints, shortcut


def _region_portals(
    region_cells: set[tuple[int, int]],
    centres: dict[tuple[int, int], tuple[float, float]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> set[tuple[int, int]]:
    """Boundary cells of the cylinder block (plus start/goal if inside)."""
    portals: set[tuple[int, int]] = set()
    for c, r in region_cells:
        if (c, r) not in centres:
            continue
        for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nb = (c + dc, r + dr)
            if nb in centres and nb not in region_cells:
                portals.add((c, r))
                break
    if start in region_cells and start in centres:
        portals.add(start)
    if goal in region_cells and goal in centres:
        portals.add(goal)
    return portals


def plan_cylinders(
    occ: OccupancyMap,
    params: Params,
    region_cells: set[tuple[int, int]] | list[tuple[int, int]],
    *,
    mode: str = "cylinders1",
) -> PlannedPath:
    """Hybrid: orthogonal minturns outside the cylinder block, fastest inside.

    Outside cells and region portals form a minturns cell graph. Crossing the
    block is a portal→portal fine-grid ``fastest`` segment.
    """
    R = {(int(c), int(r)) for c, r in region_cells}
    if not R:
        planned = plan_cell_centers_minturns(occ, params)
        return PlannedPath(
            mode=mode,
            waypoints=planned.waypoints,
            path_cells=planned.path_cells,
            length_mm=planned.length_mm,
        )

    centres, start, goal = _maze_centres(occ, params)
    cell_mm = float(params.maze["cell_size_mm"])
    turn_penalty = float(params.planning.get("turn_penalty", 0.1))

    portals = _region_portals(R, centres, start, goal)
    interior = {cr for cr in R if cr in centres} - portals
    nodes = {cr for cr in centres if cr not in interior}
    if start not in nodes or goal not in nodes:
        raise ValueError("start/goal not free for cylinder hybrid plan")

    dirs = ((-1, 0), (1, 0), (0, -1), (0, 1))
    fine_cache: dict[
        tuple[tuple[int, int], tuple[int, int]],
        tuple[float, list[Waypoint], list[tuple[int, int]]],
    ] = {}

    def _portal_fine(
        a: tuple[int, int], b: tuple[int, int]
    ) -> tuple[float, list[Waypoint], list[tuple[int, int]]]:
        key = (a, b)
        if key in fine_cache:
            return fine_cache[key]
        wps, cells = _fine_segment_mm(occ, params, centres[a], centres[b])
        cost = _path_length_mm(wps) / cell_mm
        fine_cache[key] = (cost, wps, cells)
        fine_cache[(b, a)] = (
            cost,
            list(reversed([Waypoint(x=w.x, y=w.y) for w in wps])),
            list(reversed(cells)),
        )
        return fine_cache[key]

    start_state = (start[0], start[1], -1)
    open_set: list[tuple[float, int, int, int]] = []
    heapq.heappush(open_set, (0.0, start[0], start[1], -1))
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {
        start_state: None
    }
    via: dict[tuple[int, int, int], object] = {start_state: "ortho"}
    g_score: dict[tuple[int, int, int], float] = {start_state: 0.0}
    closed: set[tuple[int, int, int]] = set()
    best_goal: tuple[int, int, int] | None = None
    goal_c, goal_r = goal

    while open_set:
        _f, c, r, d = heapq.heappop(open_set)
        state = (c, r, d)
        if state in closed:
            continue
        closed.add(state)
        if (c, r) == goal:
            best_goal = state
            break

        g = g_score[state]

        for nd, (dc, dr) in enumerate(dirs):
            nc, nr = c + dc, r + dr
            if (nc, nr) not in nodes:
                continue
            if not _edge_free(occ, centres[(c, r)], centres[(nc, nr)]):
                continue
            step = 1.0 + (turn_penalty if d >= 0 and nd != d else 0.0)
            tentative = g + step
            nstate = (nc, nr, nd)
            if nstate in closed:
                continue
            if tentative < g_score.get(nstate, float("inf")):
                came_from[nstate] = state
                via[nstate] = "ortho"
                g_score[nstate] = tentative
                h = abs(goal_c - nc) + abs(goal_r - nr)
                heapq.heappush(open_set, (tentative + h, nc, nr, nd))

        if (c, r) in portals:
            for p in portals:
                if p == (c, r):
                    continue
                try:
                    cost, _wps, _cells = _portal_fine((c, r), p)
                except (RuntimeError, ValueError):
                    continue
                tentative = g + cost
                nstate = (p[0], p[1], -1)
                if nstate in closed:
                    continue
                if tentative < g_score.get(nstate, float("inf")):
                    came_from[nstate] = state
                    via[nstate] = ("fine", (c, r), p)
                    g_score[nstate] = tentative
                    h = abs(goal_c - p[0]) + abs(goal_r - p[1])
                    heapq.heappush(open_set, (tentative + h, p[0], p[1], -1))

    if best_goal is None:
        raise RuntimeError("No cylinder hybrid path found")

    states: list[tuple[int, int, int]] = []
    node: tuple[int, int, int] | None = best_goal
    while node is not None:
        states.append(node)
        node = came_from.get(node)
    states.reverse()

    waypoints: list[Waypoint] = []

    def _append_cell(cr: tuple[int, int]) -> None:
        x, y = centres[cr]
        if (
            waypoints
            and abs(waypoints[-1].x - x) < 1e-6
            and abs(waypoints[-1].y - y) < 1e-6
        ):
            return
        waypoints.append(Waypoint(x=x, y=y))

    def _append_fine(wps: list[Waypoint], _cells: list[tuple[int, int]]) -> None:
        for wp in wps:
            if (
                waypoints
                and abs(waypoints[-1].x - wp.x) < 1e-6
                and abs(waypoints[-1].y - wp.y) < 1e-6
            ):
                continue
            waypoints.append(wp)

    _append_cell((states[0][0], states[0][1]))
    for i in range(1, len(states)):
        st = states[i]
        how = via.get(st, "ortho")
        if isinstance(how, tuple) and how[0] == "fine":
            _a, a, b = how
            _cost, wps, cells = _portal_fine(a, b)
            _append_fine(wps, cells)
        else:
            _append_cell((st[0], st[1]))

    # Debug polyline must follow waypoints 1:1 (fine LOS cells are denser/mismatched).
    path_cells = [_mm_to_occ(occ, wp.x, wp.y) for wp in waypoints]
    return PlannedPath(
        mode=mode,
        waypoints=waypoints,
        path_cells=path_cells,
        length_mm=_path_length_mm(waypoints),
    )


def plan_cylinders_prefer_ortho(
    occ: OccupancyMap,
    params: Params,
    region_cells: set[tuple[int, int]] | list[tuple[int, int]],
    *,
    mode: str = "cylinders8",
) -> tuple[PlannedPath, str]:
    """Prefer full orthogonal minturns (including the cylinder section).

    If cell-centre minturns can reach the goal under the current inflation /
    cylinder pad, use that (orthogonal even inside the cylinder block).
    Otherwise fall back to the hybrid: minturns outside + fine through portals.

    Returns ``(planned, via)`` where via is ``\"minturns\"`` or
    ``\"hybrid-fallback\"``.
    """
    try:
        planned = plan_cell_centers_minturns(occ, params)
        if planned.waypoints:
            return (
                PlannedPath(
                    mode=mode,
                    waypoints=planned.waypoints,
                    path_cells=planned.path_cells,
                    length_mm=planned.length_mm,
                ),
                "minturns",
            )
    except (RuntimeError, ValueError):
        pass
    return plan_cylinders(occ, params, region_cells, mode=mode), "hybrid-fallback"


def _empty_plan(mode: str) -> PlannedPath:
    return PlannedPath(mode=mode, waypoints=[], path_cells=[], length_mm=0.0)


def _try_plan(fn, occ: OccupancyMap, params: Params, mode: str) -> PlannedPath:
    """Run a planner; return an empty path on failure (fat padding, etc.)."""
    try:
        return fn(occ, params)
    except (RuntimeError, ValueError) as exc:
        print(f"  {mode}: failed ({exc})")
        return _empty_plan(mode)


def plan_all(
    occ: OccupancyMap, params: Params
) -> tuple[PlannedPath, PlannedPath, PlannedPath, PlannedPath]:
    return (
        _try_plan(plan_cell_centers, occ, params, "cell_centers"),
        _try_plan(plan_cell_centers_minturns, occ, params, "cell_centers_minturns"),
        _try_plan(plan_fastest, occ, params, "fastest"),
        _try_plan(plan_fastest_minturns, occ, params, "fastest_minturns"),
    )


def _draw_path_on_inflated(
    occ: OccupancyMap,
    planned: PlannedPath,
    colour: tuple[int, int, int],
) -> np.ndarray:
    scale = max(1, int(round(2.0 / occ.cell_mm)))
    vis = np.zeros((*occ.inflated.shape, 3), dtype=np.uint8)
    vis[occ.inflated == 0] = (40, 40, 40)
    vis[occ.inflated != 0] = (200, 200, 200)
    # Polyline through waypoints so markers and line stay 1:1 (path_cells can be denser).
    if planned.waypoints:
        pts = [_mm_to_occ(occ, wp.x, wp.y) for wp in planned.waypoints]
    else:
        pts = list(planned.path_cells)
    for i in range(len(pts) - 1):
        cv2.line(vis, pts[i], pts[i + 1], colour, 1, cv2.LINE_AA)
    for p in pts:
        cv2.circle(vis, p, 2, (0, 255, 255), -1, cv2.LINE_AA)
    return cv2.resize(
        vis,
        (vis.shape[1] * scale, vis.shape[0] * scale),
        interpolation=cv2.INTER_NEAREST,
    )


def save_path_debug(
    planned: PlannedPath,
    occ: OccupancyMap,
    output_dir: str | Path,
    *,
    prefix: str,
) -> None:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    colour = {
        "cell_centers": (0, 220, 0),
        "cell_centers_minturns": (0, 255, 180),
        "fastest": (0, 128, 255),
        "fastest_minturns": (255, 128, 0),
        "cylinders1": (255, 0, 200),
        "cylinders2": (200, 0, 255),
        "cylinders3": (255, 80, 160),
        "cylinders4": (180, 60, 220),
        "cylinders5": (220, 40, 180),
        "cylinders6": (160, 20, 200),
        "cylinders7": (140, 0, 160),
    }.get(planned.mode, (0, 220, 0))
    cv2.imwrite(str(out / f"{prefix}.png"), _draw_path_on_inflated(occ, planned, colour))

    payload = {
        "mode": planned.mode,
        "length_mm": planned.length_mm,
        "waypoints_mm": [[wp.x, wp.y] for wp in planned.waypoints],
    }
    with open(out / f"{prefix}.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)


def save_cell_picker(
    occ: OccupancyMap,
    params: Params,
    output_dir: str | Path,
    *,
    prefix: str = "09_cells",
) -> None:
    """Label maze cell centres on the raw occupancy map (col,row for path picks)."""
    maze = params.maze
    path_cfg = params.path
    cols = int(maze["grid_cols"])
    rows = int(maze["grid_rows"])
    cell_mm = float(maze["cell_size_mm"])
    start = tuple(int(v) for v in path_cfg["start_cell"])
    goal = tuple(int(v) for v in path_cfg["goal_cell"])

    # Upscale raw occupancy for readable labels
    scale = max(1, int(round(2.0 / occ.cell_mm)))
    base = np.zeros((*occ.occupancy.shape, 3), dtype=np.uint8)
    base[occ.occupancy == 0] = (245, 245, 245)
    base[occ.occupancy != 0] = (30, 30, 30)
    vis = cv2.resize(
        base,
        (base.shape[1] * scale, base.shape[0] * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    def mm_to_vis(x_mm: float, y_mm: float) -> tuple[int, int]:
        gx, gy = _mm_to_occ(occ, x_mm, y_mm)
        return (gx * scale, gy * scale)

    for r in range(rows):
        for c in range(cols):
            cx_mm, cy_mm = _cell_center_mm(c, r, cell_mm)
            px, py = mm_to_vis(cx_mm, cy_mm)
            free = _free_at_mm(occ, cx_mm, cy_mm, inflated=False)
            if (c, r) == start:
                colour = (0, 200, 0)
            elif (c, r) == goal:
                colour = (0, 140, 255)
            elif free:
                colour = (220, 80, 0)
            else:
                colour = (80, 80, 200)
            cv2.circle(vis, (px, py), 10, colour, -1, cv2.LINE_AA)
            label = f"{c},{r}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.putText(
                vis,
                label,
                (px - tw // 2, py - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (20, 20, 20) if free else (200, 200, 200),
                1,
                cv2.LINE_AA,
            )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"{prefix}.png"), vis)
