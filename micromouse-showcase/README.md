# Micromouse 3D Showcase

An interactive, scroll-driven 3D presentation of the UNSW Micromouse. The application is built with React, TypeScript, Three.js, and React Three Fiber, then packaged as a static Nginx site.

The production container is self-contained: it does not need a database, robot connection, internet connection, or Node.js runtime.

For implementation details, see the [system architecture](docs/architecture/README.md). For the CAD-to-GLB workflow, see [FUSION_TO_WEB_WORKFLOW.md](FUSION_TO_WEB_WORKFLOW.md).

**[Open the live GitHub Pages showcase](https://randomrunt.github.io/UNSWMicromouseDemo/)**

## Hosted deployment

The GitHub Actions workflow deploys the static site to GitHub Pages after the test and build jobs pass on a push to `main`. It can also be run manually with `workflow_dispatch`. The hosted build sets the Vite base path to `/UNSWMicromouseDemo/`, while local development and Docker builds use `/`.

The same workflow publishes the container image to GitHub Container Registry on successful pushes to `main`:

```text
ghcr.io/randomrunt/micromouse-showcase:latest
ghcr.io/randomrunt/micromouse-showcase:<commit-sha>
```

## Run with Docker Compose

### Requirements

- Docker Desktop on Windows or macOS, or Docker Engine on Linux
- Docker Compose v2 (`docker compose`)
- A modern browser with WebGL and hardware acceleration enabled

Make sure Docker Desktop or the Docker daemon is running before continuing.

### Start the showcase

From the `micromouse-showcase` directory:

```powershell
docker compose up --build -d
```

Open:

```text
http://localhost:8080
```

The first build downloads the Node and Nginx base images, installs the locked npm dependencies, and creates the production bundle. Later starts reuse Docker's build cache.

### Check its status

```powershell
docker compose ps
docker compose logs -f showcase
```

The container includes a health endpoint:

```powershell
Invoke-WebRequest http://localhost:8080/health
```

On macOS or Linux, the equivalent command is:

```bash
curl --fail http://localhost:8080/health
```

A healthy response is `ok`.

### Stop or restart it

```powershell
docker compose stop
docker compose start
```

To stop and remove the container and its Compose network:

```powershell
docker compose down
```

The built image remains cached and can be reused.

### Rebuild after changing the code or model

```powershell
docker compose up --build -d
```

For a completely fresh image build:

```powershell
docker compose build --no-cache
docker compose up -d
```

Static assets are cached by Nginx for seven days. After replacing an asset with the same filename, perform a hard refresh in the browser.

## Run without Compose

Build the image:

```powershell
docker build -t micromouse-showcase .
```

Run it:

```powershell
docker run --name micromouse-showcase -d -p 8080:80 micromouse-showcase
```

Stop and remove that container:

```powershell
docker stop micromouse-showcase
docker rm micromouse-showcase
```

## Change the host port

The Compose mapping is `8080:80`, where `8080` is the host port and `80` is the port inside the container. If port 8080 is occupied, edit `docker-compose.yml`, for example:

```yaml
ports:
  - "3000:80"
```

The showcase would then be available at `http://localhost:3000`.

## Use it on another device

The Nginx server listens on all host interfaces. A device on the same network can open:

```text
http://HOST_IP:8080
```

Replace `HOST_IP` with the showcase computer's local IP address. The host firewall must allow inbound TCP traffic on port 8080. Avoid exposing this static demo directly to the public internet without an HTTPS reverse proxy.

## 3D model selection

The checked-in digital twin is stored at this path, which the site checks at startup:

```text
public/models/micromouse.glb
```

- If the file is available, the application loads the exported digital twin.
- If it is absent, the deterministic procedural Micromouse is displayed.

Place or replace the GLB before building the Docker image, then rebuild the container. The interactive object names must match the contract in [the architecture documentation](docs/architecture/README.md#3d-model-contract).

## Showcase operation

- Scroll through all six story chapters.
- Select **Auto scroll** to advance one chapter every six seconds and loop continuously; select **Stop** to return to manual navigation.
- Select components from the component index to inspect them.
- Use the top-right switch to choose light or graphite-grey dark mode. The choice is stored in that browser.
- The final chapter enables orbit and zoom controls.
- After 90 seconds without pointer, keyboard, or scroll activity, the kiosk returns to the first chapter.

For an unattended display, start the container before the event and open `http://localhost:8080` in the browser's full-screen or kiosk mode. The Compose service uses `restart: unless-stopped`, so Docker restarts the container after a machine or daemon restart.

## Local development and testing

Docker is the recommended production path. For development with hot reload:

```powershell
npm ci
npm run dev
```

Open `http://localhost:5173`.

Available verification commands:

```powershell
npm run build
npm test
npx playwright install chromium
npm run test:e2e
```

To reproduce the GitHub Pages build locally:

```powershell
$env:VITE_BASE_PATH = '/UNSWMicromouseDemo/'
npm run build
```

## Troubleshooting

### Docker cannot connect to its API

Start Docker Desktop and wait until it reports that the engine is running. On Windows, ensure Docker Desktop is using Linux containers. Then retry `docker compose up --build -d`.

### Port 8080 is already in use

Change the host side of the port mapping in `docker-compose.yml`, then recreate the service:

```powershell
docker compose down
docker compose up --build -d
```

### A code or model change is not visible

Rebuild with `docker compose up --build -d`, then hard-refresh the page. If necessary, use the no-cache rebuild commands above.

### The 3D scene is blank or slow

Enable hardware acceleration in the browser, update the graphics driver, and verify WebGL is supported. The experience is responsive, but a discrete or recent integrated GPU is recommended for the showcase display.

### Inspect the deployed files or Nginx logs

```powershell
docker compose exec showcase ls -la /usr/share/nginx/html
docker compose logs --tail=200 showcase
```

## Container design

The `Dockerfile` uses two stages:

1. `node:24-alpine` runs `npm ci` and `npm run build`.
2. `nginx:1.29-alpine` receives only the generated `dist/` files and the Nginx configuration.

The final image does not contain npm development dependencies or the Node development server. Nginx provides SPA fallback routing, immutable caching for versioned assets, and the `/health` endpoint.
