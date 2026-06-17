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
