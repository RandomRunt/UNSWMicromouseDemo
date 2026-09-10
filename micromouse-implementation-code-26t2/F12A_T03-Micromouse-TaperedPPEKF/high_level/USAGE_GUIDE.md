# Maze Image Processing Guide

This program converts an overhead photograph of the micromouse maze into:

- detected poles, walls, shell, and optional cylinders;
- a collision/occupancy map;
- several planned routes; and
- C++ header files that can be included in the robot firmware.

## 1. Open the project directory

In PowerShell:

```powershell
cd "C:\Users\cando\Desktop\Micromouse 26T2\UNSWMicromouseDemoDay\Best_Micromouse_Implementations_26T2\F12A_T03-Micromouse-TaperedPPEKF\high_level"
```

All commands below assume that PowerShell is in this directory.

## 2. Install Python dependencies

Python 3.10 or newer is recommended. Create a virtual environment so the
packages do not interfere with other Python projects:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

If PowerShell blocks activation, either adjust its execution policy or run the
virtual environment's Python directly:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

The required packages are NumPy, OpenCV, PyYAML, and SciPy.

## 3. Prepare the photograph

Use a `.png`, `.jpg`, or `.jpeg` image. For the best result:

- photograph the complete maze from above;
- keep all four coloured orientation markers visible;
- keep the cyan pole tops visible and in focus;
- avoid cropping the outside frame;
- minimise glare, deep shadows, and objects around the maze; and
- use the original-resolution image rather than a compressed screenshot.

The orientation markers are expected to identify these corners:

| Marker | Maze corner |
|---|---|
| Pink | Top-left |
| Orange | Top-right |
| Yellow | Bottom-right |
| Purple | Bottom-left |

The program corrects perspective, so the camera does not need to be perfectly
centred. However, a reasonably high overhead view gives more reliable pole and
wall detection.

## 4. Process one image

The safest command for processing only one selected photograph is:

```powershell
python main.py --image "..\my_maze.png" --skip-all
```

An absolute path can also be used:

```powershell
python main.py --image "C:\path\to\my_maze.png" --skip-all
```

`--skip-all` is important when testing one image. Without it, the program also
processes every sibling image whose filename begins with `maze` or `cylinder`.

To put results in a separate directory:

```powershell
python main.py --image "..\my_maze.png" --output "outputs\my_maze" --skip-all
```

## 5. Select the image through `param.yaml`

Instead of passing `--image` every time, edit:

```yaml
image:
  path: ../my_maze.png
```

The path is resolved relative to `param.yaml`. Then run:

```powershell
python main.py --skip-all
```

The command-line `--image` option overrides this setting without changing the
file.

## 6. Choose ordinary-maze or cylinder detection

For a normal wall maze, set this near the top of `param.yaml`:

```yaml
enable_cylinder_detection: false
```

For a maze containing free-standing black cylinders, use:

```yaml
enable_cylinder_detection: true
```

Cylinder detection should not normally be enabled for an ordinary maze because
dark circular objects or shadows could be added as obstacles.

## 7. Set the route endpoints

The maze cells use zero-based `[column, row]` coordinates. Column numbers
increase from left to right and row numbers increase from top to bottom.

Configure the start and destination in `param.yaml`:

```yaml
path:
  start_cell: [3, 7]
  goal_cell: [4, 5]
```

After a successful run, open `outputs/09_cells.png` to see the cell coordinates
drawn on the map. Adjust the two settings and rerun if necessary.

## 8. Understand the processing stages

The pipeline runs in this order:

1. Load the colour photograph with OpenCV.
2. Detect cyan pole tops using HSV colour thresholds.
3. Detect the outer frame and coloured orientation markers.
4. Perform a coarse perspective correction using the outer corners.
5. Fit the detected poles to the expected 10 by 10 lattice and perform a fine
   perspective correction.
6. Test every possible line between adjacent poles for a wall.
7. Detect black cylinders when enabled.
8. Combine inner walls, poles, cylinders, and the outer shell.
9. Build and inflate an occupancy grid for the robot's footprint.
10. Plan several paths and export the chosen path and wall representation.

## 9. Check the generated outputs

By default, results are written to `outputs`. The most useful diagnostic files
are:

| Output | What to check |
|---|---|
| `01_poles_mask.png` | White pixels should cover the cyan pole tops only. |
| `01_poles_overlay.png` | Red circles should sit on real maze poles. |
| `02_corners_overlay.png` | The four outer corners and markers should be correct. |
| `03_warp.png` | The maze should have a roughly top-down orientation. |
| `04_fine_warp.png` | Poles should form a regular horizontal/vertical grid. |
| `05_walls_overlay.png` | Red segments are detected walls; green segments are open. |
| `05_walls.yaml` | Machine-readable list of detected walls. |
| `05b_cylinders_overlay.png` | Detected cylinders, when enabled. |
| `07_all_walls_overlay.png` | Combined inner walls, poles, and shell. |
| `08_occupancy.png` | Raw obstacle map. |
| `08_occupancy_inflated.png` | Space blocked after allowing for robot clearance. |
| `09_cells.png` | Cell coordinates used for choosing start and goal. |
| `09_path_*.png` | Visualisations of the alternative planned routes. |

The firmware-facing outputs are normally written as:

- `../waypoints_planned.hpp`
- `../walls_planned.hpp`

These files are overwritten by a successful full run. Review the path image
before compiling and uploading the firmware.

## 10. Process a folder of named images

When `--skip-all` is omitted, the program processes the selected image first and
then scans that image's directory for:

- `maze*.png`, `maze*.jpg`, and `maze*.jpeg`, with cylinders disabled; and
- `cylinder*.png`, `cylinder*.jpg`, and `cylinder*.jpeg`, with cylinders enabled.

For example:

```text
photos/
  maze1.png
  maze2.jpg
  cylinder1.png
```

Run:

```powershell
python main.py --image "..\photos\maze1.png"
```

Batch results are written below `outputs/all/<image-name>/`. A result summary is
written to `outputs/all/summary.tsv`. One failed image does not stop the rest of
the batch.

## 11. Re-plan without rerunning computer vision

If the saved vision and occupancy outputs are already correct, rerun only path
planning with:

```powershell
python main.py --path-only
```

This is useful after changing route-related settings such as `start_cell`,
`goal_cell`, or planning penalties.

To rewrite the exported waypoint header from existing planned-path files:

```powershell
python main.py --export-only
```

An export mode can be selected explicitly, for example:

```powershell
python main.py --export-only --mode cylinders3
```

## 12. Troubleshooting order

Always find the first incorrect diagnostic stage; later stages depend on it.

### Poles are missing or false poles appear

Inspect `01_poles_mask.png` and `01_poles_overlay.png`. Check lighting and cyan
visibility first. If tuning is required, the relevant `param.yaml` values are
under `poles`, especially `hsv_lo`, `hsv_hi`, `min_area`, and `max_area`.

### The maze is rotated, mirrored, or badly warped

Inspect `02_corners_overlay.png`. Confirm all four coloured markers are visible
and are placed according to the expected colour-to-corner mapping. A bad corner
stage will also make `03_warp.png` incorrect.

### The coarse warp looks right but the grid is distorted

Inspect `04_fine_warp.png`. This usually indicates missing/false pole detections
or an incorrect lattice assignment. Fix the pole stage before changing wall
thresholds.

### Walls are missing or appear where gaps should be

First confirm that `04_fine_warp.png` is correct. Then inspect
`05_walls_overlay.png`. Wall sensitivity is controlled by the `walls` section of
`param.yaml`, including `dark_ridge_threshold`, `dark_tail_threshold`, and the
enhancement thresholds. Change these carefully because increasing sensitivity
can introduce false walls.

### Cylinders are missing or false cylinders appear

Inspect `05b_cylinders_mask.png` and its overlay. Confirm cylinder detection is
enabled. Relevant settings are in the `cylinders` section, including
`dark_threshold`, radius limits, circularity, and ring contrast.

### No route is found

Check `08_occupancy_inflated.png`. A false wall, false cylinder, excessive
padding, or an invalid start/goal cell may completely block the route. Correct
the earliest bad detection instead of forcing the planner through an obstacle.

## Quick-start checklist

1. Put a clear overhead photo near the project.
2. Activate `.venv` and install `requirements.txt` once.
3. Set cylinder detection appropriately.
4. Run `python main.py --image "<photo>" --skip-all`.
5. Check outputs `01`, `02`, `04`, `05`, `08`, and `09` in order.
6. Confirm the chosen path is collision-free.
7. Compile the firmware using the generated header files.
