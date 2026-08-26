from __future__ import annotations

import argparse
import re
from dataclasses import replace
from pathlib import Path

import cv2
import yaml

from combine_walls import combine_walls, save_combined_walls
from corners import detect_corners, save_corners_debug
from cylinders import (
    cylinder_region_cells,
    detect_cylinders,
    filter_near_poles,
    load_cylinders_from_yaml,
    save_cylinders_debug,
)
from export import export_walls_header, export_waypoints_snippet, select_export_path
from occupancy import OccupancyMap, build_occupancy, load_occupancy, save_occupancy_debug
from plan_path import (
    PlannedPath,
    _empty_plan,
    load_planned_path,
    plan_all,
    plan_cylinders,
    plan_cylinders_prefer_ortho,
    plan_fastest_minturns,
    plan_manual_cells,
    save_cell_picker,
    save_path_debug,
)
from poles import (
    Params,
    _resolve_path,
    detect_poles,
    filter_pole_cluster,
    filter_poles_inside_frame,
    image_pixel_scale,
    load_params,
    replace_pole_points,
    save_poles_debug,
    scale_params_for_image,
)
from shell import build_shell, load_shell_from_yaml, save_shell_debug
from walls import detect_walls, load_wall_detection, load_walls_from_yaml, save_walls_debug
from warp import (
    filter_poles_lacking_cyan,
    fine_warp_from_poles,
    load_fine_warp,
    recover_poles_on_lattice,
    save_fine_warp_debug,
    save_warp_debug,
    warp_from_outer_corners,
)


def _params_with_cylinder_pad(params: Params, pad_mm: float) -> Params:
    map_cfg = dict(params.map)
    map_cfg["cylinder_padding_mm"] = float(pad_mm)
    return replace(params, map=map_cfg)


def _cylinder_pad_variants(params: Params) -> list[tuple[str, float]]:
    """Named hybrid path variants: cylinders1@50 … cylinders7@0."""
    plan = params.planning
    raw = plan.get(
        "cylinder_path_pads_mm",
        {
            "cylinders1": 50,
            "cylinders2": 40,
            "cylinders3": 30,
            "cylinders4": 20,
            "cylinders5": 10,
            "cylinders6": 5,
            "cylinders7": 0,
        },
    )
    if isinstance(raw, dict):
        items = [(str(k), float(v)) for k, v in raw.items()]
    else:
        items = [(f"cylinders{i + 1}", float(pad)) for i, pad in enumerate(raw)]
    # Prefer larger pad first for primary occupancy.
    items.sort(key=lambda kv: -kv[1])
    return items


def _cylinder_path_prefix(base: str, mode: str) -> str:
    """09_path_cylinders + cylinders1 → 09_path_cylinders1."""
    base = str(base)
    if base.endswith("cylinders") and mode.startswith("cylinders"):
        return base + mode.removeprefix("cylinders")
    return f"{base}_{mode}"


_MODE_PREFIX_ALIASES = {
    "centers": "path_centers_prefix",
    "cell_centers": "path_centers_prefix",
    "orthogonal": "path_centers_prefix",
    "minturns": "path_minturns_prefix",
    "cell_centers_minturns": "path_minturns_prefix",
    "min_turns": "path_minturns_prefix",
    "fastest": "path_fastest_prefix",
    "shortest": "path_fastest_prefix",
    "fastest_minturns": "path_fastest_minturns_prefix",
    "fast_minturns": "path_fastest_minturns_prefix",
    "straight": "path_fastest_minturns_prefix",
    "straights": "path_fastest_minturns_prefix",
}


def _normalize_export_mode(mode: str) -> str:
    mode = str(mode).lower().strip()
    if mode in ("cylinders", "path_cylinders", "hybrid", "cylinder"):
        return "cylinders1"
    return mode


def _export_path_yaml(output_dir: Path, params: Params, mode: str) -> Path:
    """Map an export mode name to the existing ``09_path_*.yaml``."""
    mode = _normalize_export_mode(mode)
    out = params.output
    if mode == "manual":
        return Path(output_dir) / "09_path_manual.yaml"
    if mode.startswith("cylinders"):
        prefix = _cylinder_path_prefix(
            out.get("path_cylinders_prefix", "09_path_cylinders"), mode
        )
        return Path(output_dir) / f"{prefix}.yaml"
    key = _MODE_PREFIX_ALIASES.get(mode)
    if key is None:
        raise ValueError(
            f"unknown export mode {mode!r} "
            "(use cylinders1..8, minturns, centers, fastest, fastest_minturns, manual)"
        )
    defaults = {
        "path_centers_prefix": "09_path_centers",
        "path_minturns_prefix": "09_path_minturns",
        "path_fastest_prefix": "09_path_fastest",
        "path_fastest_minturns_prefix": "09_path_fastest_minturns",
    }
    prefix = out.get(key, defaults[key])
    return Path(output_dir) / f"{prefix}.yaml"


def export_from_outputs(
    params: Params,
    output_dir: Path,
    *,
    mode: str | None = None,
) -> Path:
    """Rewrite waypoint headers from existing ``09_path_*.yaml`` (no re-plan)."""
    output_dir = Path(output_dir)
    cyl_yaml = output_dir / f"{params.output.get('cylinders_prefix', '05b_cylinders')}.yaml"
    has_cylinders = False
    region_cells: list[tuple[int, int]] | None = None
    if cyl_yaml.is_file():
        cyl = yaml.safe_load(cyl_yaml.read_text(encoding="utf-8")) or {}
        has_cylinders = bool(cyl.get("cylinders") or cyl.get("count"))
        cells = (cyl.get("region_cells") or {}).get("cells")
        if cells:
            region_cells = [(int(c), int(r)) for c, r in cells]

    if mode is None:
        plan = params.planning
        if has_cylinders:
            mode = str(plan.get("cylinder_export_mode", "cylinders1"))
        else:
            mode = str(plan.get("export_mode", "minturns"))
    mode = _normalize_export_mode(mode)

    yaml_path = _export_path_yaml(output_dir, params, mode)
    if not yaml_path.is_file():
        raise FileNotFoundError(
            f"no planned path {yaml_path.name}; run the full pipeline first"
        )
    planned = load_planned_path(yaml_path)
    if not planned.waypoints:
        raise RuntimeError(
            f"mode {mode!r} has no waypoints in {yaml_path.name}"
        )

    walls_yaml = output_dir / f"{params.output.get('walls_prefix', '05_walls')}.yaml"
    walls = load_walls_from_yaml(walls_yaml) if walls_yaml.is_file() else []

    print(
        f"Export-only {yaml_path.name} → {planned.mode} "
        f"({len(planned.waypoints)} wps, {planned.length_mm:.1f} mm)"
    )
    header = export_waypoints_snippet(
        planned,
        params,
        output_dir,
        prefix=params.output.get("waypoints_prefix", "10_waypoints"),
        region_cells=region_cells,
        walls=walls,
    )
    print(f"  sketch header: {header}")
    return header


def build_occupancy_for_path(
    fine,
    walls,
    shell,
    params: Params,
    cylinders: list,
    *,
    require_path: bool = True,
) -> tuple[OccupancyMap, Params, float | None]:
    """Build occupancy; with cylinders, pick the largest named pad that paths.

    Named pads default to cylinders1..7 at 50/40/30/20/10/5/0 mm. Falls back through
    that list until ``plan_cylinders`` succeeds.
    """
    if not cylinders:
        occ = build_occupancy(fine, walls, shell, params, cylinders=cylinders)
        return occ, params, None

    variants = _cylinder_pad_variants(params)
    if not require_path:
        pad = variants[0][1]
        trial = _params_with_cylinder_pad(params, pad)
        occ = build_occupancy(fine, walls, shell, trial, cylinders=cylinders)
        print(f"  cylinder pad {pad:g} mm (occupancy-only, no path search)")
        return occ, trial, pad

    poles_ij = [(int(i), int(j)) for (i, j) in fine.poles_ij]
    region = cylinder_region_cells(
        cylinders, params, walls=walls, poles_ij=poles_ij
    )
    last_err: Exception | None = None
    for name, pad in variants:
        trial = _params_with_cylinder_pad(params, pad)
        occ = build_occupancy(fine, walls, shell, trial, cylinders=cylinders)
        try:
            if region is None:
                plan_fastest_minturns(occ, trial)
            else:
                planned = plan_cylinders(
                    occ, trial, region.cells(), mode=name
                )
                if not planned.waypoints:
                    raise RuntimeError(f"{name} produced no waypoints")
        except (RuntimeError, ValueError) as exc:
            last_err = exc
            print(f"  {name} pad {pad:g} mm: no path ({exc})")
            continue
        print(f"  {name} pad {pad:g} mm: OK (primary occupancy)")
        return occ, trial, pad

    detail = f": {last_err}" if last_err else ""
    raise RuntimeError(
        "No cylinder padding left a feasible path"
        f" (tried {variants}){detail}"
    )


def plan_cylinder_pad_variants(
    fine,
    walls,
    shell,
    params: Params,
    cylinders: list,
    region_cells: list[tuple[int, int]],
) -> dict[str, tuple[OccupancyMap, PlannedPath, float]]:
    """Plan hybrid paths at each named pad. Returns mode → (occ, planned, pad)."""
    out: dict[str, tuple[OccupancyMap, PlannedPath, float]] = {}
    for name, pad in _cylinder_pad_variants(params):
        trial = _params_with_cylinder_pad(params, pad)
        occ = build_occupancy(fine, walls, shell, trial, cylinders=cylinders)
        try:
            planned = plan_cylinders(occ, trial, region_cells, mode=name)
            if not planned.waypoints:
                raise RuntimeError("empty path")
            print(
                f"  {name}: {len(planned.waypoints)} wps, "
                f"length = {planned.length_mm:.1f} mm  (pad {pad:g} mm)"
            )
        except (RuntimeError, ValueError) as exc:
            print(f"  {name}: failed at pad {pad:g} mm ({exc})")
            planned = _empty_plan(name)
        out[name] = (occ, planned, pad)
    return out


def _natural_key(path: Path) -> list:
    parts = re.split(r"(\d+)", path.stem)
    return [int(p) if p.isdigit() else p.lower() for p in parts]


def discover_batch_images(image_dir: Path) -> tuple[list[Path], list[Path]]:
    """Find maze* / cylinder* photos next to the configured image."""
    maze_imgs: list[Path] = []
    cylinder_imgs: list[Path] = []
    for path in sorted(image_dir.iterdir(), key=_natural_key):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            continue
        name = path.name.lower()
        if name.startswith("maze"):
            maze_imgs.append(path)
        elif name.startswith("cylinder"):
            cylinder_imgs.append(path)
    return maze_imgs, cylinder_imgs


def run_pipeline(
    params: Params,
    image_path: Path,
    output_dir: Path,
    *,
    occupancy_only: bool = False,
) -> None:
    """Run the vision → occupancy (+ optional plan/export) pipeline."""
    pole_prefix = params.output["pole_prefix"]
    corner_prefix = params.output.get("corner_prefix", "02_corners")
    warp_prefix = params.output.get("warp_prefix", "03_warp")
    fine_prefix = params.output.get("fine_warp_prefix", "04_fine_warp")
    walls_prefix = params.output.get("walls_prefix", "05_walls")
    shell_prefix = params.output.get("shell_prefix", "06_shell")
    all_walls_prefix = params.output.get("all_walls_prefix", "07_all_walls")
    cylinders_prefix = params.output.get("cylinders_prefix", "05b_cylinders")
    occupancy_prefix = params.output.get("occupancy_prefix", "08_occupancy")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    scale = image_pixel_scale(
        image.shape,
        ref_width_px=float(params.poles.get("ref_width_px", 1920)),
    )
    if scale > 1.0 + 1e-9:
        print(
            f"  pixel-scale ×{scale:.3f} for {image.shape[1]}x{image.shape[0]} "
            f"(ref {params.poles.get('ref_width_px', 1920)}px wide)"
        )
        params = scale_params_for_image(params, image.shape)

    print(f"Detect poles: {image_path}")
    poles = detect_poles(image, params)
    expect = params.maze["pole_grid_cols"] * params.maze["pole_grid_rows"] - 12
    print(f"  poles = {len(poles.poles_px)} (expect ~{expect})")

    cluster_k = float(params.poles.get("cluster_filter_spacings", 6.5))
    kept_px, keep_mask = filter_pole_cluster(
        poles.poles_px, max_radius_spacings=cluster_k
    )
    dropped = int((~keep_mask).sum())
    if dropped:
        poles = replace_pole_points(poles, image, kept_px, params)
        print(
            f"  cluster filter: kept {len(kept_px)} / {len(keep_mask)}"
            f" (dropped {dropped}, k={cluster_k:g})"
        )
    else:
        print(f"  cluster filter: kept all {len(kept_px)} (k={cluster_k:g})")
    save_poles_debug(poles, output_dir, prefix=pole_prefix)

    print("Detect corners")
    corners = detect_corners(image, poles.poles_px, params)
    save_corners_debug(corners, output_dir, prefix=corner_prefix)
    print(f"  outer corners = {len(corners.outer_corners_px)}")
    print(f"  diagonal endpoints = {len(corners.diagonal_endpoints_px)}")
    print(f"  colour markers = {sorted(corners.colour_centroids_px)}")

    frame_margin = float(params.poles.get("frame_filter_margin_px", 20.0))
    kept_px, keep_mask = filter_poles_inside_frame(
        poles.poles_px,
        corners.outer_corners_px,
        margin_px=frame_margin,
    )
    dropped = int((~keep_mask).sum())
    if dropped:
        poles = replace_pole_points(poles, image, kept_px, params)
        save_poles_debug(poles, output_dir, prefix=pole_prefix)
        print(
            f"  frame filter: kept {len(kept_px)} / {len(keep_mask)}"
            f" (dropped {dropped}, margin={frame_margin:g}px)"
        )
    else:
        print(f"  frame filter: kept all {len(kept_px)} (margin={frame_margin:g}px)")

    print("Warp (coarse, outer corners)")
    coarse = warp_from_outer_corners(
        image,
        corners.outer_corners_px,
        corners.colour_centroids_px,
        params,
    )
    save_warp_debug(coarse, output_dir, prefix=warp_prefix)
    print(f"  output = {coarse.output_size_px}x{coarse.output_size_px}")

    print("Warp (fine, pole lattice)")
    fine = fine_warp_from_poles(image, poles.poles_px, coarse, params)
    w, h = fine.canvas_size_px
    print(f"  poles indexed = {len(fine.poles_ij)}")
    print(f"  output = {w}x{h}")
    print(f"  pixels_per_mm = {fine.pixels_per_mm:.4f}")

    fine, n_recovered = recover_poles_on_lattice(fine, params)
    if n_recovered:
        print(
            f"  lattice recover: +{n_recovered} poles "
            f"(now {len(fine.poles_ij)})"
        )
    else:
        print("  lattice recover: none")

    fine, n_dropped = filter_poles_lacking_cyan(fine, params)
    if n_dropped:
        print(
            f"  cyan validate: dropped {n_dropped} ghost poles "
            f"(now {len(fine.poles_ij)})"
        )
    else:
        print("  cyan validate: kept all")
    save_fine_warp_debug(fine, output_dir, prefix=fine_prefix)

    # Cylinders before walls so dark disks can be blanked out of ridge scores
    # (otherwise a cylinder overlapping a lattice edge looks like a wall).
    cylinder_list = []
    if params.enable_cylinder_detection:
        print("Detect cylinders")
        cyl = detect_cylinders(image, fine, params)
        reject_mm = float(params.cylinders.get("reject_near_pole_mm", 0.0))
        if reject_mm > 0:
            maze_cell = float(params.maze["cell_size_mm"])
            poles_mm = [
                (float(i) * maze_cell, float(j) * maze_cell)
                for (i, j) in fine.poles_ij
            ]
            before = len(cyl.cylinders)
            kept = filter_near_poles(cyl.cylinders, poles_mm, min_dist_mm=reject_mm)
            if len(kept) != before:
                cyl.cylinders = kept
                print(
                    f"  near-pole filter: kept {len(kept)} / {before}"
                    f" (min_dist={reject_mm:g} mm)"
                )
        save_cylinders_debug(
            cyl, output_dir, prefix=cylinders_prefix, fine=fine, params=params
        )
        cylinder_list = cyl.cylinders
        print(f"  cylinders = {len(cylinder_list)}")
        for i, c in enumerate(cylinder_list):
            print(
                f"    [{i}] ({c.x_mm:.1f}, {c.y_mm:.1f}) mm"
                f"  r={c.radius_mm:.1f}  circ={c.circularity:.2f}"
            )
        if cylinder_list:
            print(f"  wrote {output_dir}/{cylinders_prefix}_region.png")
        print(f"  wrote {output_dir}/{cylinders_prefix}_*")
    else:
        print("Detect cylinders: skipped (enable_cylinder_detection=false)")

    print("Detect walls")
    walls = detect_walls(fine, params, cylinders=cylinder_list)
    save_walls_debug(walls, output_dir, params, prefix=walls_prefix)
    if params.poles.get("assume_all_present"):
        print("  assume_all_present: sampling on full lattice")
    print(f"  wall segments = {len(walls.walls)}")
    print(f"  wrote {output_dir}/{walls_prefix}_*")

    print("Build outer shell")
    shell = build_shell(fine, params)
    save_shell_debug(shell, output_dir, prefix=shell_prefix)
    print(f"  shell segments = {len(shell.segments)}")
    print(
        f"  inner span = {shell.meta.get('derived_inner_span_mm')} mm"
        f"  edge gap derived = {shell.meta.get('derived_edge_gap_mm'):.2f} mm"
        f" (measured {shell.meta.get('outer_pole_to_wall_mm')})"
    )
    print(
        f"  corner gap derived = {shell.meta.get('derived_corner_gap_mm'):.2f} mm"
        f" (measured {shell.meta.get('corner_pole_to_diagonal_mm')})"
    )
    print(f"  wrote {output_dir}/{shell_prefix}_*")

    print("Combine walls")
    combined = combine_walls(fine, walls, shell, params)
    save_combined_walls(
        combined,
        walls,
        shell,
        fine,
        output_dir,
        params,
        prefix=all_walls_prefix,
        cylinders=cylinder_list or None,
    )
    print(
        f"  inner = {combined.inner_count}, poles = {combined.pole_count}, "
        f"shell = {combined.shell_count}"
    )
    print(
        f"  wrote {output_dir}/{all_walls_prefix}_overlay.png"
        f", {all_walls_prefix}_overlay_measured.png"
        f", {all_walls_prefix}_schematic_measured.png"
    )
    if cylinder_list:
        print(f"  wrote {output_dir}/{all_walls_prefix}_overlay_region.png")

    print("Build occupancy")
    occ, params, cyl_pad = build_occupancy_for_path(
        fine,
        walls,
        shell,
        params,
        cylinder_list,
        require_path=not occupancy_only,
    )
    save_occupancy_debug(occ, output_dir, params, prefix=occupancy_prefix)
    h, w = occ.occupancy.shape
    print(f"  grid = {w}x{h} @ {occ.cell_mm} mm/cell")
    print(f"  inflate_shape = {params.map.get('inflate_shape', 'diamond')}")
    if cyl_pad is not None:
        print(f"  cylinder_padding_mm = {cyl_pad:g} (max feasible)")
    print(f"  occupied = {int(occ.occupancy.sum())}, inflated = {int(occ.inflated.sum())}")
    print(f"  wrote {output_dir}/{occupancy_prefix}_*")

    if occupancy_only:
        return

    plan_and_export(
        params,
        output_dir,
        occ,
        walls,
        shell,
        fine,
        cylinder_list,
    )


def plan_and_export(
    params: Params,
    output_dir: Path,
    occ: OccupancyMap,
    walls,
    shell,
    fine,
    cylinder_list: list,
) -> None:
    """Plan all path modes and rewrite robot headers. No vision."""
    output_dir = Path(output_dir)
    centers_prefix = params.output.get("path_centers_prefix", "09_path_centers")
    minturns_prefix = params.output.get("path_minturns_prefix", "09_path_minturns")
    fastest_prefix = params.output.get("path_fastest_prefix", "09_path_fastest")
    fastest_minturns_prefix = params.output.get(
        "path_fastest_minturns_prefix", "09_path_fastest_minturns"
    )
    cylinders_path_prefix = params.output.get(
        "path_cylinders_prefix", "09_path_cylinders"
    )
    cells_prefix = params.output.get("cells_prefix", "09_cells")
    waypoints_prefix = params.output.get("waypoints_prefix", "10_waypoints")
    walls_export_prefix = params.output.get("walls_export_prefix", "11_walls")

    has_cylinders = len(cylinder_list) > 0

    print("Plan paths")
    save_cell_picker(occ, params, output_dir, prefix=cells_prefix)
    centers, minturns, fastest, fastest_minturns = plan_all(occ, params)
    save_path_debug(centers, occ, output_dir, prefix=centers_prefix)
    save_path_debug(minturns, occ, output_dir, prefix=minturns_prefix)
    save_path_debug(fastest, occ, output_dir, prefix=fastest_prefix)
    save_path_debug(
        fastest_minturns, occ, output_dir, prefix=fastest_minturns_prefix
    )
    cylinder_paths: dict[str, PlannedPath] = {}
    export_region_cells: list[tuple[int, int]] | None = None
    if has_cylinders:
        poles_ij = [(int(i), int(j)) for (i, j) in fine.poles_ij]
        region = cylinder_region_cells(
            cylinder_list, params, walls=walls, poles_ij=poles_ij
        )
        if region is not None:
            export_region_cells = list(region.cells())
            print(
                f"  cylinder region = {region.width}x{region.height}"
                f" @ ({region.col0},{region.row0})"
            )
            variants = plan_cylinder_pad_variants(
                fine,
                walls,
                shell,
                params,
                cylinder_list,
                export_region_cells,
            )
            wrote: list[str] = []
            for name, (v_occ, planned, _pad) in variants.items():
                prefix = _cylinder_path_prefix(cylinders_path_prefix, name)
                save_path_debug(planned, v_occ, output_dir, prefix=prefix)
                cylinder_paths[name] = planned
                wrote.append(f"{prefix}_*")
            # cylinders8: minturns through the cylinder section when pad allows,
            # else same hybrid as cylinders1..7 (uses primary occupancy pad).
            try:
                planned8, via8 = plan_cylinders_prefer_ortho(
                    occ, params, export_region_cells, mode="cylinders8"
                )
                if not planned8.waypoints:
                    raise RuntimeError("empty path")
                print(
                    f"  cylinders8: {len(planned8.waypoints)} wps, "
                    f"length = {planned8.length_mm:.1f} mm  ({via8})"
                )
            except (RuntimeError, ValueError) as exc:
                print(f"  cylinders8: failed ({exc})")
                planned8 = _empty_plan("cylinders8")
            prefix8 = _cylinder_path_prefix(cylinders_path_prefix, "cylinders8")
            save_path_debug(planned8, occ, output_dir, prefix=prefix8)
            cylinder_paths["cylinders8"] = planned8
            wrote.append(f"{prefix8}_*")
            print(f"  wrote {', '.join(wrote)}")
    print(f"  cell picker = {output_dir}/{cells_prefix}.png")
    print(
        f"  cell_centers:      {len(centers.waypoints)} wps, "
        f"length = {centers.length_mm:.1f} mm"
    )
    print(
        f"  minturns:          {len(minturns.waypoints)} wps, "
        f"length = {minturns.length_mm:.1f} mm"
    )
    print(
        f"  fastest:           {len(fastest.waypoints)} wps, "
        f"length = {fastest.length_mm:.1f} mm"
    )
    print(
        f"  fastest_minturns:  {len(fastest_minturns.waypoints)} wps, "
        f"length = {fastest_minturns.length_mm:.1f} mm"
    )
    print(
        f"  wrote {output_dir}/{centers_prefix}_* , {minturns_prefix}_* , "
        f"{fastest_prefix}_* , {fastest_minturns_prefix}_*"
    )

    print("Export waypoints for robot")
    manual = plan_manual_cells(occ, params)
    if manual is not None:
        chosen = manual
        save_path_debug(manual, occ, output_dir, prefix="09_path_manual")
        print(
            f"  manual_cells: {len(manual.waypoints)} wps, "
            f"length = {manual.length_mm:.1f} mm"
        )
    else:
        chosen = select_export_path(
            centers,
            minturns,
            fastest,
            fastest_minturns,
            params,
            has_cylinders=has_cylinders,
            cylinder_paths=cylinder_paths,
        )
    if not chosen.waypoints:
        raise RuntimeError(
            f"Export mode '{chosen.mode}' produced no waypoints"
            + (" (cylinders present)" if has_cylinders else "")
        )
    snippet = export_waypoints_snippet(
        chosen,
        params,
        output_dir,
        walls=walls,
        prefix=waypoints_prefix,
        region_cells=export_region_cells,
    )
    export_label = (
        f"{chosen.mode} [cylinder]" if has_cylinders else chosen.mode
    )
    print(f"  mode = {export_label} ({len(chosen.waypoints)} waypoints)")
    print(f"  sketch header: {snippet}")

    print("Export wall geometry for robot")
    wall_header = export_walls_header(
        walls, shell, fine, params, output_dir, prefix=walls_export_prefix
    )
    print(f"  wall header: {wall_header}")


def run_path_only(params: Params, output_dir: Path) -> None:
    """Skip vision + occupancy: load outputs, replan paths, rewrite headers."""
    output_dir = Path(output_dir)
    fine_prefix = params.output.get("fine_warp_prefix", "04_fine_warp")
    walls_prefix = params.output.get("walls_prefix", "05_walls")
    shell_prefix = params.output.get("shell_prefix", "06_shell")
    cylinders_prefix = params.output.get("cylinders_prefix", "05b_cylinders")
    occupancy_prefix = params.output.get("occupancy_prefix", "08_occupancy")

    print("Path-only: skip vision, load occupancy + walls")
    fine = load_fine_warp(output_dir, params, prefix=fine_prefix)
    walls_path = output_dir / f"{walls_prefix}.yaml"
    if not walls_path.is_file():
        raise FileNotFoundError(
            f"missing {walls_path.name}; run the full pipeline first"
        )
    walls = load_wall_detection(walls_path)
    shell_path = output_dir / f"{shell_prefix}.yaml"
    if not shell_path.is_file():
        raise FileNotFoundError(
            f"missing {shell_path.name}; run the full pipeline first"
        )
    shell = load_shell_from_yaml(shell_path, overlay_bgr=fine.warped_bgr)

    cylinder_list = []
    cyl_path = output_dir / f"{cylinders_prefix}.yaml"
    if params.enable_cylinder_detection and cyl_path.is_file():
        cylinder_list = load_cylinders_from_yaml(cyl_path)

    occ, meta = load_occupancy(output_dir, prefix=occupancy_prefix)
    pad = meta.get("cylinder_padding_mm")
    if pad is not None:
        params = _params_with_cylinder_pad(params, float(pad))
    h, w = occ.occupancy.shape
    print(
        f"  poles = {len(fine.poles_ij)}, walls = {len(walls.walls)}, "
        f"cylinders = {len(cylinder_list)}"
    )
    print(
        f"  occupancy {w}x{h} @ {occ.cell_mm} mm/cell"
        + (f", cylinder pad {float(pad):g} mm" if pad is not None else "")
    )

    plan_and_export(
        params,
        output_dir,
        occ,
        walls,
        shell,
        fine,
        cylinder_list,
    )


def generate_all_occupancy(
    params: Params,
    *,
    config_path: Path,
    image_dir: Path,
    all_dir: Path,
) -> None:
    """Build occupancy for every maze*/cylinder* image under ``all_dir``.

    Uses settings from ``params`` / ``param.yaml`` but does not modify the file.
    ``maze*`` runs with cylinders off; ``cylinder*`` runs with cylinders on.
    Failures are logged and skipped so one bad photo cannot abort the rest.
    """
    maze_imgs, cylinder_imgs = discover_batch_images(image_dir)
    jobs: list[tuple[Path, bool]] = [
        *((p, False) for p in maze_imgs),
        *((p, True) for p in cylinder_imgs),
    ]
    if not jobs:
        print(f"All-images occupancy: no maze*/cylinder* photos in {image_dir}")
        return

    all_dir = Path(all_dir)
    all_dir.mkdir(parents=True, exist_ok=True)
    summary_path = all_dir / "summary.tsv"
    rows = ["stem\tkind\tcylinders\tstatus\tnotes"]

    print(
        f"\nAll-images occupancy → {all_dir}  "
        f"({len(maze_imgs)} mazes, {len(cylinder_imgs)} cylinders)"
    )
    for image_path, with_cylinders in jobs:
        stem = image_path.stem
        kind = "cylinder" if with_cylinders else "maze"
        out = all_dir / stem
        out.mkdir(parents=True, exist_ok=True)
        # Keep sketch headers out of the selected-maze paths in param.yaml.
        out_cfg = dict(params.output)
        out_cfg["dir"] = str(out)
        out_cfg["waypoints_header"] = str(out / "waypoints_planned.hpp")
        out_cfg["walls_header"] = str(out / "walls_planned.hpp")
        job_params = replace(
            params,
            enable_cylinder_detection=with_cylinders,
            output=out_cfg,
        )
        print(
            f"\n----- {stem} ({kind}, cylinders={with_cylinders}) → {out} -----"
        )
        try:
            run_pipeline(
                job_params,
                image_path,
                out,
                occupancy_only=True,
            )
            rows.append(f"{stem}\t{kind}\t{with_cylinders}\tOK\t")
        except Exception as exc:  # noqa: BLE001 — batch must keep going
            note = str(exc).replace("\t", " ").replace("\n", " ")[:200]
            print(f"  FAILED: {exc}")
            rows.append(f"{stem}\t{kind}\t{with_cylinders}\tFAIL\t{note}")

    summary_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    ok = sum(1 for r in rows[1:] if "\tOK\t" in r)
    fail = len(rows) - 1 - ok
    print(
        f"\nAll-images occupancy done: {ok} ok, {fail} failed → {summary_path}"
        f"  (param.yaml image path unchanged: "
        f"{_resolve_path(config_path, params.image['path'])})"
    )


def main() -> None:
    root = Path(__file__).parent
    parser = argparse.ArgumentParser(description="Micromouse high_level")
    parser.add_argument(
        "--config",
        type=Path,
        default=root / "param.yaml",
        help="Path to param.yaml",
    )
    parser.add_argument(
        "--image",
        type=Path,
        default=None,
        help="Override image path from config",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Override output directory from config",
    )
    parser.add_argument(
        "--skip-all",
        action="store_true",
        help="Skip the always-on outputs/all occupancy batch for every photo",
    )
    parser.add_argument(
        "--export-only",
        action="store_true",
        help="Rewrite waypoints_planned.hpp from existing 09_path_*.yaml (no re-plan)",
    )
    parser.add_argument(
        "--path-only",
        action="store_true",
        help="Skip vision + occupancy; replan paths from saved outputs",
    )
    parser.add_argument(
        "--mode",
        default=None,
        help="Export mode for --export-only (e.g. cylinders3). Default: param.yaml",
    )
    args = parser.parse_args()

    params = load_params(args.config)
    image_path = args.image or _resolve_path(args.config, params.image["path"])
    output_dir = args.output or (args.config.parent / params.output["dir"])

    if args.export_only:
        export_from_outputs(params, Path(output_dir), mode=args.mode)
        return

    if args.path_only:
        run_path_only(params, Path(output_dir))
        return

    run_pipeline(params, Path(image_path), Path(output_dir), occupancy_only=False)

    if not args.skip_all:
        # Sibling photos of the configured image; write under <output>/all/.
        generate_all_occupancy(
            params,
            config_path=args.config,
            image_dir=Path(image_path).resolve().parent,
            all_dir=Path(output_dir) / "all",
        )


if __name__ == "__main__":
    main()
