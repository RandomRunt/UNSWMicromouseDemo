# AGENTS.md

This file gives coding agents the repository-specific context needed to work safely and efficiently in `micromouse-showcase`. It applies to this directory and everything below it.

## Project goal

This project is a deterministic, scroll-driven 3D presentation of the UNSW Micromouse. It must remain useful as an unattended showcase and must still work without a robot, backend, database, or runtime internet connection.

Preserve these product qualities when making changes:

- The six-chapter story remains understandable with mouse, touch, or keyboard input.
- The semantic HTML experience remains usable when the WebGL scene is unavailable.
- The production output remains a static Vite bundle suitable for both Nginx and GitHub Pages.
- The procedural Micromouse remains a functional fallback when the GLB cannot load.
- Kiosk behavior, reduced-motion support, responsive layout, and light/dark themes remain intact.

## Repository boundaries

This app lives inside a larger worktree. Unless the task explicitly says otherwise, only edit files inside this `micromouse-showcase/` directory. Do not modify sibling projects or parent-level files.

Before editing, inspect `git status --short` and preserve all pre-existing user changes. Do not revert, overwrite, or reformat unrelated work. `dist/`, `node_modules/`, `test-results/`, and `playwright-report/` are generated and should not be committed or edited by hand.

## Technology

- React 19 and TypeScript with strict checking
- Three.js through React Three Fiber and Drei
- Vite for development and static builds
- Vitest for unit and contract tests
- Playwright plus axe-core for browser and accessibility checks
- Nginx and Docker Compose for the production kiosk image

Use the existing npm toolchain and `package-lock.json`. Prefer `npm ci` for a clean install. Do not introduce a new package, state library, build tool, or formatter unless the task genuinely requires it.

## Important files

- `src/App.tsx`: composition root and shared UI/scene state
- `src/components/`: semantic overlay, labels, loading UI, and the canvas boundary
- `src/scene/`: React Three Fiber scene, model, lighting, camera, and animation code
- `src/story/chapters.ts`: canonical six-chapter content
- `src/story/`: scroll, reduced-motion, and kiosk hooks
- `src/config/components.ts`: interactive component metadata and GLB mesh-name contract
- `src/config/assets.ts`: base-path-safe public asset URLs
- `src/styles/global.css`: theme tokens, layout, responsive rules, and motion preferences
- `src/types/showcase.ts`: shared domain types
- `public/models/micromouse.glb`: web-ready digital twin
- `tests/asset-contract.test.ts`: model and motion contract tests
- `tests/e2e/showcase.spec.ts`: desktop/mobile journey and accessibility coverage
- `docs/architecture/README.md`: implemented system design and invariants
- `FUSION_TO_WEB_WORKFLOW.md`: CAD-to-GLB preparation and export process

Read the architecture document before changing cross-cutting state, chapter flow, model loading, deployment, or the interaction contract. Read the Fusion workflow before changing the GLB or its object names.

## Architecture invariants

Keep shared state in `App.tsx` unless complexity clearly justifies a different boundary. The DOM overlay and WebGL scene should receive explicit typed props and share behavior through that state.

The WebGL canvas is visual enhancement, is `aria-hidden`, and is not the sole route to content or interaction. Any selectable 3D component needs an equivalent keyboard-accessible DOM control and visible selection feedback.

The model URL must continue to derive from `import.meta.env.BASE_URL`; do not hard-code `/models/...` or the GitHub Pages host. Local and Docker builds use `/`, while the hosted build uses `/UNSWMicromouseDemo/`.

The checked-in GLB and procedural model are interchangeable implementations. Interactive GLB nodes use stable, unique, lowercase ASCII names with underscores. When adding, removing, or renaming an interactive component, update all relevant parts together:

1. The GLB object name or procedural mesh.
2. `componentDefinitions` and the `ComponentId` type.
3. Selection/highlight behavior in the scene.
4. Accessible DOM controls and presentation copy.
5. Contract and end-to-end tests.
6. Architecture and asset-workflow documentation when the contract changes.

Chapter changes usually span `chapters.ts`, camera targets or motion, chapter-specific scene visibility, overlay behavior, and Playwright coverage. Treat chapter indices as a coordinated contract rather than isolated magic numbers.

## Coding conventions

- Follow the existing TypeScript and React style: two-space indentation, single quotes, semicolons, trailing commas in multiline structures, and named exports except for the root `App` component.
- Keep strict types. Do not add `any`, broad type assertions, or TypeScript suppression comments to bypass a design problem.
- Prefer small functional components, focused hooks, explicit props, and data-driven configuration.
- Keep React render functions free of side effects. Clean up timers, animation work, listeners, fetches, and other browser resources in effect cleanup functions.
- Avoid per-frame React state updates in Three.js code. Use refs and `useFrame` for continuously animated scene state, and reuse geometries/materials where practical.
- Keep animation deterministic and respect `prefers-reduced-motion` for new transitions.
- Reuse the CSS custom properties and responsive patterns in `global.css`; avoid hard-coded colors when an appropriate theme token exists.
- Preserve test-facing roles, accessible names, and `data-testid` values unless the task deliberately changes their contract and updates the tests.
- Add comments for coordinate systems, timing constraints, asset contracts, or non-obvious browser/Three.js behavior, not for code that is already self-explanatory.

There is currently no lint or formatting script. Do not claim one was run. TypeScript validation happens as part of the production build.

## Accessibility and interaction

Accessibility is a core architecture constraint, not a final polish pass. For user-facing changes:

- Use semantic HTML and native controls before custom interaction primitives.
- Ensure every control is reachable and operable by keyboard, has an accurate accessible name, and retains a visible focus state.
- Do not encode state or meaning only through color, animation, or the 3D scene.
- Keep headings, chapter announcements, skip navigation, pressed/expanded states, and component details meaningful to screen readers.
- Verify both desktop and mobile layouts and avoid obscuring story copy with canvas overlays or component labels.
- Maintain adequate contrast in both themes and avoid a fully black dark-mode background.

## Asset and performance guidance

Do not edit binary CAD or GLB assets unless the request is specifically about those assets. Keep source CAD under `cad/` and browser-ready assets under `public/`; paths in `cad/` are excluded from the Docker build.

When changing the scene, consider GPU and kiosk hardware limits. Avoid unnecessary high-poly geometry, large textures, per-frame allocation, excessive lights, or remounting the canvas between chapters. The experience should degrade gracefully if the model request fails or WebGL performance is limited.

## Verification

Run the smallest relevant check while iterating, then the broader checks appropriate to the completed change:

```powershell
npm test
npm run build
npm run test:e2e
```

- `npm test` runs the Vitest model/motion contract tests.
- `npm run build` runs the strict TypeScript project build and Vite production build.
- `npm run test:e2e` starts Vite through Playwright and tests desktop and Pixel 7 projects.
- If Chromium is unavailable locally, install it with `npx playwright install chromium` and report if installation could not be performed.

For a GitHub Pages-specific asset or routing change, also verify the repository base path:

```powershell
$env:VITE_BASE_PATH = '/UNSWMicromouseDemo/'
npm run build
Remove-Item Env:VITE_BASE_PATH
```

Do not run every expensive browser test for a documentation-only change. For behavior or visual changes, add or update focused tests and run the applicable Playwright project(s). Check browser console errors during scene changes because WebGL and asset failures may not appear in unit tests.

## Documentation and handoff

Keep documentation aligned with implemented behavior. Update `README.md` for setup, operation, or deployment changes; update `docs/architecture/README.md` for system contracts and state flow; update `FUSION_TO_WEB_WORKFLOW.md` for model-export requirements.

At handoff, summarize the user-visible result, list the checks actually run and their outcomes, and call out any remaining risk or unverified step. Never say tests passed unless they were run successfully in the current workspace.
