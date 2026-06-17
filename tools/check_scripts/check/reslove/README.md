# Sunray dependency resolve scripts

This directory stores optional dependency resolve scripts for `check.sh`.

Expected layout:

```text
tools/check_scripts/check/reslove/
  livox_ros_driver2/
    01_install_apt_deps.sh
    02_build_livox_sdk2.sh
  realsense2_camera/
    01_install_realsense_deps.sh
```

Rules:

- The subdirectory name should match the module name in `modules.yaml`.
- Scripts are executed in lexical order.
- Scripts are launched with `bash` from the repository root.
- Keep scripts small and explicit. Prefer one concern per script.
- Source `../common.sh` when a script needs apt, sudo, or dry-run helpers.
- Respect `SUNRAY_DRY_RUN=true`; print commands without changing the system.
- Respect `SUNRAY_ASSUME_YES=true`; avoid interactive prompts in resolver scripts.
- Scripts must be idempotent where practical and return non-zero on failure.
- Print a clear message before modifying system paths such as `/usr/local` or `/etc`.
- `sunray_apt_install` first skips apt work for packages already installed by
  `dpkg-query`. This keeps an unrelated third-party apt source failure from
  breaking modules whose packages are already present.
- If packages are missing, `sunray_apt_install` runs `apt-get update` at most
  once per resolver run. By default, update failures are warnings and
  installation is still attempted.
- Set `SUNRAY_SKIP_APT_UPDATE=true` or pass `--skip-apt-update` to avoid update
  entirely when the local apt index is known to be usable.
- Set `SUNRAY_STRICT_APT_UPDATE=true` or pass `--strict-apt-update` to make
  update failures fatal.
- Resolver scripts must not automatically modify `/etc/apt/sources.list` or
  `/etc/apt/sources.list.d`. Disable or repair broken third-party sources only
  through an explicit user maintenance flow.

The directory name `reslove` is retained for compatibility. New code may also
support a correctly spelled `resolve` directory, but existing scripts should not
be moved without a compatibility plan.
