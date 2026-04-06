# Getting started

## Prerequisites

- **Docker** — install [Docker Desktop](https://docs.docker.com/get-docker/) (macOS/Windows) or Docker Engine (Linux)

All other tools (west, Zephyr SDK 1.0.1, CMake, Python, etc.) are provided by
the Zephyr developer Docker image.

## Clone & start the dev container

```bash
mkdir 2dof-workspace && cd 2dof-workspace
git clone <repo-url> 2dof-tilt-platform-fw
cd 2dof-tilt-platform-fw
./scripts/start_container.sh
```

## Initialize the west workspace (first time only)

Inside the container:

```bash
west init -l .
cd /workdir
west update
cd 2dof-tilt-platform-fw
```

After `west update`, the workspace should look like this:

```
2dof-workspace/              ← workspace root (.west/ lives here)
├── .west/
├── 2dof-tilt-platform-fw/   ← this repo
├── zephyr/
└── modules/
    ├── hal/nordic/
    └── lib/cmsis_6/
```

## Build

```bash
west build -b xiao_ble/nrf52840 samples/bringup --pristine
```

## Flash

The XIAO nRF52840 uses a UF2 bootloader. Double-tap the reset button to
enter the bootloader, then:

```bash
west flash
```

Alternatively, drag-and-drop `build/zephyr/zephyr.uf2` onto the mounted
USB drive.
