# Build Infrastructure Overview

This repository uses a shared Docker base image plus per-workspace images, with a consistent build and packaging flow. The goal is to keep workspace dependencies isolated while still sharing a common, reusable base.

## High-Level Architecture

- **Base image (top-level `Dockerfile`)**
  - Contains shared, stable dependencies (ROS base, build tooling, rosdep, vcstool, etc.).
  - Built once per run by the top-level `build.sh`.

- **Workspace images (one per workspace)**
  - Each workspace has its own `Dockerfile` that starts from the base image.
  - Workspace-specific dependencies are installed here (via `rosdep`, apt, pip, etc.).
  - This keeps unrelated dependencies out of other workspaces.

- **Workspace build scripts (`<ws>/build.sh`)**
  - Each workspace defines how it is built (colcon, make/cmake, custom flows, etc.).
  - The script builds the workspace image, then runs the build inside a container with the workspace mounted.

- **Packaging**
  - Workspace build scripts produce build artifacts in their own folders (e.g., `install/`, `build/`).
  - `package.sh` stages artifacts into `dist/<ws>/` and creates a single tarball with selected workspaces plus an installer helper.

## Configuration

All Docker-related variables are centralized in `docker_env.sh`:

- `ROS_DISTRO`
- `PLATFORM`
- `BASE_IMAGE`
- `*_IMAGE` per workspace

Override any of them by exporting variables before running a script. Example:

```bash
ROS_DISTRO_OVERRIDE=humble ./build.sh --ws <workspace>
```

## Build Flow

**Top-level build** (`build.sh`):

1. Builds the base image (every run).
2. Calls each workspace build script (from `--all` or `--ws`). The `--all` flag uses the WS_* workspace scanner (`scripts/list_workspaces.sh`).

**Workspace build** (`<ws>/build.sh`):

1. Builds the workspace image from its local `Dockerfile`.
2. Runs the workspace build inside a container.
3. Leaves build artifacts in the workspace (e.g., `install/`, `build/`).

You can list detected workspaces by running:

```bash
scripts/list_workspaces.sh
```

## Packaging & Distribution

**Packaging is separate** from builds.

- `package.sh` stages `dist/<ws>/` and produces a tarball:
  - `dist/bundle_YYYYMMDD_HHMMSS.tar.gz`
- The bundle includes:
  - All selected workspaces under a single directory
  - `install_all.sh` to run dependency installers (`install_deps.sh`) in each workspace folder
  - `setup.sh` / `setup.bash` to source installed workspaces (and `/opt/ros/$ROS_DISTRO` if set)
  - Workspace extras (e.g., `WS_Mini_Bot/config`, `WS_Mini_Bot/scripts`) when present
- For WS_* workspaces without ROS `package.xml` files, generated `install_deps.sh` skips `rosdep` automatically.
- Device packaging auto-ships via scp (uses `package.conf` `ssh_config` by default, falls back to `~/.ssh/config`). Examples:
- `./package.sh --device Mini` (auto-ships to host `Mini`)
- `./package.sh --device Mini --ship-path ~/bundles/`

**Typical use:**

```bash
./build.sh --all
./package.sh --all
```

The resulting tarball can be shipped to another device, unpacked, and installed using the included scripts.

## Adding New Workspaces

To add a new workspace to this system using the WS_* convention:

1. Create a top-level folder named `WS_<name>`.
2. Add `<ws>/build.sh` and `<ws>/src/` (these are required for auto-detection).
3. Add `<ws>/Dockerfile` if the workspace uses the Docker-based build flow.
4. Ensure `<ws>/build.sh` writes build artifacts to `<ws>/install/` if you want compiled outputs shipped in bundles.
5. Optional: `<ws>/config/`, `<ws>/scripts/`, and `<ws>/install/` will be packaged automatically.

No changes are required to the top-level scripts if you follow the WS_* structure. For non-WS_* workspaces or special packaging needs, add a custom rule in `package.sh`.
