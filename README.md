# 2DOF Tilt Platform — Firmware

Zephyr RTOS firmware for the **Seeed XIAO nRF52840 Sense** controlling a
2-DOF motorized tilt platform.

## Quick start

```bash
mkdir 2dof-workspace && cd 2dof-workspace
git clone <repo-url> 2dof-tilt-platform-fw
cd 2dof-tilt-platform-fw
./scripts/start_container.sh

# Inside the container (first time only):
west init -l . && cd /workdir && west update && cd 2dof-tilt-platform-fw

# Build & flash:
west build -b xiao_ble/nrf52840 samples/bringup --pristine
west flash
```

## Documentation

Full documentation can be found in [docs/](docs/index.md).

## License

Apache-2.0
