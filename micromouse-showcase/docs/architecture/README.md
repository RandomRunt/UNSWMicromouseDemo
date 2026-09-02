# Micromouse Showcase System Architecture

This document describes the implemented software architecture of the Micromouse 3D Showcase. The system is a browser-only, deterministic presentation deployed as static files. It has no backend service, database, live telemetry dependency, or runtime internet requirement.

For operating instructions, see the [project README](../../README.md). For the 3D asset preparation process, see the [Fusion-to-web workflow](../../FUSION_TO_WEB_WORKFLOW.md).

## System context

```mermaid
flowchart LR
    V[Visitor] -->|scroll, click, keyboard| B[Browser]
    B -->|HTTP| H{Static host}
    H --> N[Nginx container]
    H --> P[GitHub Pages]
    N -->|HTML, CSS, JS, GLB| B
    P -->|HTML, CSS, JS, GLB| B
    B --> D[React DOM story and controls]
    B --> W[React Three Fiber / WebGL scene]
    D <-->|shared React state| W
```

Nginx serves the application and its assets. After the initial download, all storytelling, rendering, interaction, theming, and kiosk behavior execute in the browser.

## Technology layers

| Layer | Technology | Responsibility |
|---|---|---|
| Deployment | Docker Compose | Builds, publishes port 8080, and restarts the service |
| Static server | Nginx | Serves the bundle, caches assets, provides SPA fallback and health checks |
| Build | Node.js, npm, Vite, TypeScript | Installs locked dependencies, type-checks, and creates `dist/` |
| UI | React | Owns application state, story content, controls, themes, and accessibility equivalents |
| 3D integration | React Three Fiber and Drei | Maps React components to the Three.js scene and provides controls/helpers |
| Rendering | Three.js and WebGL | Renders the robot, lighting, maze, beams, grid, and animations |
| Styling | CSS | Provides layout, responsive behavior, light/dark tokens, and reduced-motion handling |
| Verification | Vitest, Playwright, axe-core | Checks the model contract, user journeys, viewports, and accessibility |

## Runtime composition

`src/App.tsx` is the composition root. It owns the small amount of shared state and passes explicit props into the DOM overlay and the WebGL scene.

```mermaid
flowchart TD
    Events[Scroll, resize, pointer, keyboard] --> Hooks[Browser hooks]
    Hooks --> App[App state]
    Asset[HEAD /models/micromouse.glb] --> App
    App --> Story[StoryOverlay and chapter sections]
    App --> Canvas[ShowcaseCanvas]
    App --> Card[ComponentLabel]
    Canvas --> Camera[CameraRig]
    Canvas --> Env[Lighting and SceneEnvironment]
    Canvas --> Model{Model available?}
    Model -->|yes| GLB[GLBMicromouse]
    Model -->|no| Proc[ProceduralMicromouse]
    Canvas --> Effects[SensorBeams and MazeDemo]
```

### Shared state

| State | Source | Consumers |
|---|---|---|
| Normalized scroll progress | `useScrollProgress` | Camera choreography and story transitions |
| Active chapter | `useScrollProgress` | Story overlay, scene effects, exploded model state, controls |
| Selected component | Component index or 3D pointer event | Model highlight, pressed state, component detail card |
| Exploration exploded | Final-chapter control | Optional teardown while orbit controls are enabled |
| Mobile 3D view | Final-chapter mobile control | Locks page scrolling and gives touch gestures to orbit controls |
| Theme | Saved preference or dark-mode default | CSS token system, canvas background, fog, grid, scene lighting |
| Reduced motion | `prefers-reduced-motion` | Camera and object animation behavior |
| GLB availability | Startup `HEAD` request | Selection between the GLB and procedural model |
| Auto-scroll enabled | Visitor control | Six-second cyclic chapter navigation |

No external state library is required because this state is local, shallow, and has a single composition root.

## Story and animation flow

The page contains six full-height semantic sections defined by `src/story/chapters.ts`:

1. Meet the Micromouse
2. What is inside?
3. How does it sense?
4. How does it think?
5. How does it move?
6. Explore it yourself

`useScrollProgress` converts `window.scrollY` into a value from 0 to 1 and derives the active chapter. Scroll updates are limited to one state update per animation frame. `CameraRig` and scene components map the current chapter to target positions and interpolate toward them in the render loop.

The optional auto-scroll control keeps its own chapter cursor. While enabled, a six-second interval scrolls to the next section and wraps from chapter six to chapter one. Visitors can stop it from the top bar. When reduced motion is requested, chapter changes are immediate rather than smooth.

The final chapter enables orbit controls automatically at the page end on desktop. On mobile, visitors use an explicit **Enter 3D view** control so touch gestures cannot compete with page scrolling; leaving the mode restores normal navigation. Earlier chapters retain authored camera positions so every visitor receives the same narrative sequence.

## 3D scene architecture

`ShowcaseCanvas` creates one persistent React Three Fiber canvas containing:

- `Lighting` for theme-aware ambient, directional, and accent lights;
- `SceneEnvironment` for fog, the floor grid, and dark-mode stars;
- `CameraRig` for chapter camera choreography and final exploration controls;
- `GLBMicromouse` or `ProceduralMicromouse` for the robot;
- `SensorBeams` for the range and inertial sensing chapter;
- `MazeDemo` for the movement chapter;
- a Suspense fallback while asynchronous scene assets load.

The WebGL canvas is marked `aria-hidden`. Equivalent component names, story text, controls, state, and selection feedback are provided in semantic HTML. This keeps the visual scene independent from the accessible interaction layer.

### 3D model contract

At startup the application sends a `HEAD` request to `/models/micromouse.glb`. A valid response selects the GLB implementation; otherwise the procedural implementation provides a fully functional fallback.

The checked-in GLB contains these stable, unique object names:

```text
mouse_root
chassis
bottom_pcb
top_pcb
battery
power_switch
microcontroller
imu
tof_left
tof_front
tof_right
oled_display
motor_driver
motor_left
motor_right
encoder_left
encoder_right
wheel_left
wheel_right
ball_caster_front
ball_caster_rear
```

`src/config/components.ts` is the presentation contract. It maps interactive mesh names to labels, descriptions, accent colors, and exploded-view offsets. Tests ensure names are unique and use lowercase ASCII with underscores.

Keeping presentation metadata outside the GLB allows the asset to be re-exported without rewriting story or UI code, provided the object names remain stable.

## Interaction and accessibility

The DOM layer provides:

- semantic chapter sections and headings;
- a keyboard-accessible component index;
- visible focus indicators;
- an `aria-live` chapter announcement;
- a skip link;
- reduced-motion support;
- light and dark themes with a persistent browser preference;
- responsive desktop and mobile layouts.

Selecting a component in the HTML index updates the same React state as selecting a 3D object. The component detail card therefore behaves consistently regardless of input method.

Chapter 02 presents a one-time component-selection hint until the visitor selects a part from either the robot or component index. Chapter 06 presents a matching orbit hint until the visitor first drags the camera or zooms inward. Both hints remain dismissed for the rest of the current showcase session.

The theme is stored under `micromouse-theme` in `localStorage`. On the first visit, dark mode is used by default. Theme state affects both CSS custom properties and WebGL scene values.

## Kiosk behavior

`useKioskReset` listens for pointer, keyboard, and scroll activity. After 90 seconds without activity it clears the selected component and returns the page to the first chapter. Auto-scroll generates scroll activity, so an actively cycling unattended tour does not trigger the idle reset.

The kiosk timer is deliberately client-side. It requires no server session and resets independently in every open browser tab.

## Build and deployment architecture

```mermaid
flowchart LR
    Source[TypeScript, CSS, public assets] --> Node[Node build stage]
    Lock[package-lock.json] -->|npm ci| Node
    Node -->|npm run build| Dist[dist directory]
    Dist --> Nginx[Nginx runtime image]
    Config[nginx.conf] --> Nginx
    Nginx -->|port 80| Host[Host port 8080]
    Dist --> Artifact[GitHub Pages artifact]
    Artifact --> Pages[GitHub Pages deployment]
```

The multi-stage `Dockerfile` separates build-time and runtime concerns:

1. The Node stage copies the package manifests, runs `npm ci`, copies the source, and runs the type-checked Vite production build.
2. The Nginx stage copies only `dist/` and `nginx.conf` into the runtime image.

The GitHub Actions workflow uses the same verified source for two publication targets after pushes to `main`: tagged container images in GitHub Container Registry and a static GitHub Pages artifact. The Pages build supplies `VITE_BASE_PATH=/UNSWMicromouseDemo/`; `src/config/assets.ts` derives the GLB URL from Vite's `BASE_URL`, so model loading works both below that repository path and at `/` in local or Nginx deployments.

Nginx behavior:

- `/health` returns `200 ok` for the Docker health check;
- unknown application paths fall back to `index.html`;
- JavaScript, CSS, model, image, and font assets receive a seven-day immutable cache policy;
- missing static assets return 404 rather than the SPA shell.

The runtime is read-only application content. There are no API endpoints and no persistent container volumes.

## Repository structure

```text
micromouse-showcase/
|-- README.md                         # Docker operation and development guide
|-- FUSION_TO_WEB_WORKFLOW.md         # CAD and GLB preparation
|-- docs/architecture/README.md       # This document
|-- Dockerfile
|-- docker-compose.yml
|-- nginx.conf
|-- public/
|   `-- models/micromouse.glb         # Exported digital twin
|-- cad/                              # Source/reference assets, not served
|-- src/
|   |-- App.tsx                       # State and application composition
|   |-- components/                   # DOM overlay and canvas boundary
|   |-- scene/                        # Three.js/R3F scene components
|   |-- story/                        # Chapter data and browser hooks
|   |-- config/components.ts          # Interactive model contract
|   |-- styles/global.css             # Theme and responsive design system
|   `-- types/                        # Shared TypeScript types
`-- tests/
    |-- asset-contract.test.ts
    `-- e2e/showcase.spec.ts
```

The `cad/` directory is excluded from the Docker build context. Only web-ready files under `public/` are copied into the Vite bundle.

## Verification strategy

The project uses two complementary test layers:

- Vitest validates the static model/component contract.
- Playwright runs the application in Chromium at desktop and mobile viewport sizes.

The browser suite checks all six chapters, console health, keyboard component inspection, critical axe accessibility violations, responsive readability, theme switching, the non-black graphite dark background, and saved theme preference.

The production build also runs TypeScript before Vite, so type errors stop Docker image creation.

## Extension points

The current system is intentionally deterministic. Likely extensions and their boundaries are:

| Extension | Recommended boundary |
|---|---|
| Live robot telemetry | Add a typed adapter hook; keep story components unaware of transport details |
| Additional chapters | Extend `chapters.ts`, camera targets, and any chapter-specific scene visibility |
| New interactive components | Add the mesh to the GLB contract and one entry in `components.ts` |
| Alternate robot model | Preserve the object-name contract or introduce a model-specific adapter |
| Alternate hosted deployment | Keep the static bundle and configure Vite's base path for the target URL |
| Analytics | Add a consent-aware browser adapter; do not couple it to rendering |

If live telemetry is introduced, the static story should remain the fallback so the showcase still operates when the robot or network is unavailable.
