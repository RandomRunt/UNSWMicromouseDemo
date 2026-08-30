# Fusion 360 to Interactive Web Model Workflow

This guide defines the asset workflow for converting the Micromouse assembly from Fusion 360 into a polished, rotatable, component-aware model for the showcase website.

## Final workflow

```text
Fusion 360
  |-- native F3D/F3Z       -> editable engineering master
  |-- STEP                 -> neutral CAD archive
  `-- FBX                  -> preferred Blender handoff
                                |
                                v
                             Blender
                         cleanup and materials
                                |
                                v
                     public/models/micromouse.glb
                                |
                                v
                       Interactive web showcase
```

If the FBX does not preserve separate components, export the important components from Fusion as individual OBJ files and import those into Blender instead.

Onshape is not part of the normal workflow. Do not rebuild the robot in Onshape just to create the web model.

## Which program owns each decision?

| Decision | Fusion 360 | Blender | Website |
|---|---:|---:|---:|
| Mechanical geometry and dimensions | Primary | Verify only | No |
| Component separation and engineering names | Primary | Final cleanup | Read names |
| Approximate component colours | Yes | Refine | No |
| Final colours and materials | Reference only | Primary | Display them |
| Metallic, roughness, textures and shading | No | Primary | Display them |
| Component pivots and exploded-view readiness | Initial structure | Primary | Animate them |
| Hover and selected highlighting | No | No | Primary |
| Labels and component descriptions | No | No | Primary |
| Final browser file | No | Export GLB | Load GLB |

The short rule is:

> Use Fusion for correct parts and approximate colours, Blender for the final appearance, and website code for interactive highlighting.

## Project file locations

Use these locations consistently:

```text
micromouse-showcase/
|-- cad/
|   |-- source/
|   |   |-- micromouse.f3d       # Native Fusion source, if self-contained
|   |   |-- micromouse.f3z       # Native Fusion archive, if externally linked
|   |   |-- micromouse.step      # Neutral CAD archive
|   |   |-- micromouse.fbx       # Preferred Blender handoff
|   |   `-- parts/               # Per-component OBJ fallback
|   |
|   |-- blender/
|   |   |-- micromouse-working.blend
|   |   `-- textures-source/
|   |
|   `-- reference/
|       |-- top.jpg
|       |-- front.jpg
|       |-- side.jpg
|       `-- component-notes.md
|
`-- public/
    |-- models/
    |   `-- micromouse.glb        # Final model used by the website
    `-- textures/                 # Only for intentionally separate textures
```

Native CAD, STEP, and Blender files are working sources. They are excluded from the production Docker image. Only the final GLB and required public textures are served to visitors.

## Stage 1: Prepare the assembly in Fusion 360

### 1.1 Preserve the engineering master

Keep the original Fusion design as the authoritative mechanical model. Do not make Blender the source for engineering changes.

- Use `.f3d` for a self-contained design.
- Use `.f3z` when the design includes external references.
- Save a STEP export as a neutral recovery/archive file.

Copy the available source files into `cad/source/`.

### 1.2 Organize the assembly

Before exporting, make the assembly easy to understand:

- Use proper Fusion components for important physical parts.
- Give every important component a meaningful name.
- Keep anything that will be clickable as a separate component or body.
- Confirm the assembled robot is correctly positioned.
- Confirm the real width, length, and height.
- Record the robot's forward direction in `cad/reference/component-notes.md`.

Important interactive parts should include, where present:

```text
chassis
top_plate
controller
motor_driver
battery
imu
lidar_left
lidar_front
lidar_right
motor_left
motor_right
wheel_left
wheel_right
encoder_left
encoder_right
```

### 1.3 Remove unnecessary complexity

Suppress or simplify details that will not be visible on a monitor:

- internal threads;
- tiny washers and fasteners;
- hidden PCB traces and internal electronics;
- embossed manufacturing text that is too small to see;
- internal motor geometry;
- duplicated or suppressed design variants.

Keep silhouette-defining features, visible connectors, sensor apertures, wheels, and recognizable electronics.

### 1.4 Add approximate colours in Fusion

Apply simple appearances that make the assembly understandable and resemble the real robot. These colours are references for Blender, not guaranteed final website materials.

Suggested categories:

| Component | Fusion appearance goal |
|---|---|
| Chassis and brackets | Approximate aluminium, printed plastic, or real chassis colour |
| PCB/controller | Representative PCB green or actual solder-mask colour |
| LiDAR and IMU housings | Dark plastic or actual housing colour |
| Motors and shafts | Metallic grey |
| Wheels | Near-black rubber |
| Battery | Actual casing or heat-shrink colour |
| Wires/connectors | Simplified real colours where visually useful |

Do not spend significant time perfecting reflections, procedural appearances, decals, or render-only effects in Fusion. They may not transfer faithfully to Blender.

### Fusion checkpoint

Do not export until:

- [ ] Important parts are separate.
- [ ] Important parts have useful names.
- [ ] Dimensions match the real robot.
- [ ] The assembled pose is correct.
- [ ] Approximate colours have been applied.
- [ ] Invisible complexity has been reduced where practical.
- [ ] Native Fusion and STEP source files have been saved.

## Stage 2: Export from Fusion 360

### 2.1 Preferred export: FBX

Use Fusion's normal **File -> Export** command and export:

```text
cad/source/micromouse.fbx
```

FBX is the first choice because it has a better chance than a geometry-only format of carrying an assembly-like object structure and useful appearance information into Blender.

Perform a test export early. Do not spend hours polishing Fusion materials before confirming that the major components survive the FBX import as separate Blender objects.

### 2.2 FBX acceptance test

Import the test FBX into Blender and answer:

- Are the chassis, wheels, motors, sensors, controller, and battery separately selectable?
- Are they still correctly positioned?
- Did useful component names survive?
- Is the physical scale recoverable and correct?
- Did approximate colours survive well enough to use as references?

If the geometry and separation are good, use the FBX workflow.

If the colours are poor but the objects are separate, continue with the FBX and rebuild the materials in Blender. Poor imported materials are not a reason to discard otherwise good geometry.

### 2.3 Fallback export: per-component OBJ

Use this fallback only if the FBX merges important components or creates an unusable structure.

For each important Fusion component:

1. Right-click the component or body.
2. Choose **Save As Mesh**.
3. Choose OBJ.
4. Set an explicit unit, preferably millimetres.
5. Export one component per file into `cad/source/parts/`.

Example:

```text
cad/source/parts/
|-- chassis.obj
|-- controller.obj
|-- battery.obj
|-- lidar_left.obj
|-- lidar_front.obj
|-- lidar_right.obj
|-- motor_left.obj
|-- motor_right.obj
|-- wheel_left.obj
`-- wheel_right.obj
```

OBJ is a fallback because it may lose hierarchy and material behavior. Separate files nevertheless guarantee that the interactive components can remain independent in Blender.

### 2.4 Formats not to use as the main handoff

- **STEP:** retain it as the engineering archive, but do not serve or directly load it in the website.
- **STL:** avoid it because it commonly loses names, hierarchy, materials, colours, and explicit units.
- **Onshape recreation:** avoid rebuilding the assembly merely to export GLB.

## Stage 3: Prepare the model in Blender

### 3.1 Create the working file

1. Import `cad/source/micromouse.fbx`, or import all OBJ fallback files.
2. Save the Blender project immediately as:

```text
cad/blender/micromouse-working.blend
```

3. Set the Blender scene to metric units.
4. Compare the imported dimensions with the real robot.
5. Correct scale and orientation before editing materials or animation pivots.

### 3.2 Verify object separation and names

Every component that the visitor can select must be a distinct Blender object. Rename the final objects using lowercase names and underscores:

```text
lidar_front
wheel_left
motor_right
```

Avoid:

```text
Body42
Component 7:1
Front Lidar Final FINAL
```

Object names drive selection and animation in the website. Material names do not identify components.

### 3.3 Set origins and pivots

- Put each wheel origin on its axle so it rotates correctly.
- Give exploded-view components sensible local origins.
- Keep the whole robot grouped beneath a `mouse_root` object.
- Apply transforms only after scale, orientation, and pivots are correct.

The exploded motion will normally be generated by website code. Blender does not need a permanent exploded animation, but a temporary exploded pose is useful for checking collisions.

### 3.4 Clean the geometry

- Remove hidden internal faces and geometry where safe.
- Merge tiny non-interactive objects that use the same material.
- Keep interactive objects separate.
- Recalculate or correct face normals.
- Check smooth and flat shading.
- Use limited decimation only where it does not damage the silhouette.

### 3.5 Finalize materials in Blender

Blender is the final authority for the model's appearance.

Use glTF-compatible **Principled BSDF** materials. Focus on:

- base colour;
- metallic value;
- roughness value;
- normal maps where genuinely useful;
- emissive details only when a real indicator or visual explanation needs them.

Suggested reusable material names:

```text
mat_chassis_aluminium
mat_pcb_green
mat_black_plastic
mat_rubber
mat_motor_metal
mat_battery
mat_sensor_aperture
```

Suggested starting points, to be adjusted visually:

| Material | Metallic | Roughness | Notes |
|---|---:|---:|---|
| Aluminium | High | Low to medium | Avoid a perfect mirror finish |
| Painted metal | Medium to high | Medium | Base colour should match the real coating |
| Plastic | Zero | Medium | Increase roughness for moulded or printed parts |
| Rubber | Zero | High | Use a near-black base rather than absolute black |
| PCB | Zero | Medium | Add textures only if they remain visible at showcase distance |
| Sensor aperture | Zero | Low to medium | Prefer a dark glossy surface over complex transparency |

Imported Fusion materials may be kept if they render correctly, but consolidate duplicates and replace anything that relies on unsupported procedural nodes. Arbitrary Blender node graphs will not necessarily export to GLB as expected.

### 3.6 Keep interaction styling out of Blender

Do not create permanent neon, hover, selected, or outlined versions of each component in Blender.

The website will apply interaction effects dynamically:

- hover: subtle brightness or outline;
- selected: stronger highlight and component label;
- focus: smooth camera movement;
- sensor demonstration: temporary beam or animated overlay.

This preserves one clean base material per physical surface.

### Blender checkpoint

- [ ] Robot dimensions are correct.
- [ ] Forward direction is known.
- [ ] Interactive components are separate objects.
- [ ] Object names follow the website contract.
- [ ] Wheels have correct axle pivots.
- [ ] Exploded components have sensible origins.
- [ ] Geometry is visually clean and reasonably optimized.
- [ ] Materials use Principled BSDF.
- [ ] Colours and surfaces resemble the real robot.
- [ ] No interaction-only highlight materials were added.

## Stage 4: Export the final GLB

Export from Blender using **File -> Export -> glTF 2.0**.

Use these initial settings:

- Format: **glTF Binary (`.glb`)**
- Include: selected or visible production objects only
- Transform: **Y Up**
- Geometry: UVs, normals, materials, and applied modifiers
- Animation: disabled initially
- Compression: disabled for the first working export

Export to:

```text
public/models/micromouse.glb
```

The website must load the GLB from:

```text
/models/micromouse.glb
```

Do not export the GLB into `src/`, `cad/`, or the repository root.

## Stage 5: Validate the GLB

### Independent validation

Before opening the website:

1. Open the GLB in a standalone glTF viewer.
2. Confirm that the full robot appears.
3. Confirm orientation and scale.
4. Confirm materials and colours.
5. Inspect the object hierarchy and names.
6. Run the Khronos glTF Validator when possible.

### Website validation

Then confirm:

- [ ] The GLB loads without an error.
- [ ] The robot can rotate smoothly.
- [ ] Every configured component can be found by name.
- [ ] Hover selects only the intended component.
- [ ] Click focus moves to the intended component.
- [ ] Wheels rotate around their axles.
- [ ] Exploded offsets move the correct objects.
- [ ] Materials look correct under the website lighting.
- [ ] The model performs acceptably on the showcase computer.

## Troubleshooting decision tree

### FBX imports as one merged object

Use Fusion's **Save As Mesh** and export the interactive components as individual OBJ files. Import them together in Blender.

### FBX objects are separate, but colours are wrong

Keep the FBX geometry. Rebuild or refine materials in Blender.

### Model is the wrong size

Stop before applying transforms. Compare one known physical dimension, correct the unit conversion, and verify the complete bounding dimensions.

### Components rotate around strange points

Correct their origins in Blender. Wheels should use their axle centres; exploded components should use stable local pivots.

### Model looks correct in Blender but wrong in the browser

Open the GLB in an independent viewer. If it is also wrong there, correct the Blender material/export. If it is correct there, inspect the website lighting or model-loading code.

### GLB is too large or performs poorly

Optimize in this order:

1. Remove invisible geometry.
2. Merge tiny non-interactive objects.
3. Reduce unnecessary texture resolution.
4. Reduce geometry that does not affect the silhouette.
5. Introduce mesh or texture compression only after the uncompressed asset works.

## Final handoff checklist

The asset is ready for website development when all of the following exist:

```text
cad/source/micromouse.f3d or micromouse.f3z
cad/source/micromouse.step
cad/source/micromouse.fbx or cad/source/parts/*.obj
cad/blender/micromouse-working.blend
public/models/micromouse.glb
cad/reference/component-notes.md
```

The decisive acceptance condition is:

> `public/models/micromouse.glb` opens correctly, preserves separately named interactive parts, resembles the real robot, and performs smoothly on the showcase computer.

## Reference documentation

- [Autodesk Fusion: export designs](https://help.autodesk.com/view/fusion360/ENU/?contextId=ASM-EXPORT-DESIGN)
- [Autodesk Fusion: Save As Mesh](https://help.autodesk.com/view/fusion360/ENU/?contextId=MESH-SAVE-AS-MESH)
- [Blender: importing and exporting files](https://docs.blender.org/manual/en/4.4/files/import_export/index.html)
- [Blender: glTF 2.0 export and materials](https://docs.blender.org/manual/en/4.4/addons/import_export/scene_gltf2.html)
- [Khronos: glTF production pipeline](https://github.khronos.org/Vulkan-Site/tutorial/latest/Advanced_glTF/Tooling_Production_Pipeline/01_introduction.html)
