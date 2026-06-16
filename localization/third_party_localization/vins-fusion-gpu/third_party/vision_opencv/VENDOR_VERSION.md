# vision_opencv Vendor Source

This directory vendors `ros-perception/vision_opencv` as plain source, not as a
git submodule.

- Upstream: https://github.com/ros-perception/vision_opencv
- ROS distro: Noetic
- Version/tag: `1.16.2`
- Commit: `08b012c038e575d7fe1d538f11235a994159dc93`
- Reason: `ros-noetic-cv-bridge` in the target system is version `1.16.2`, and
  the ROS Index lists `vision_opencv`, `cv_bridge`, `image_geometry`, and
  `opencv_tests` for Noetic at version `1.16.2`.

The internal `.git` directory was removed intentionally so this code is tracked
directly by the Sunray repository.

Only `cv_bridge` is meant to be built by the Sunray VINS workspace. The
`image_geometry`, `opencv_tests`, and `vision_opencv` metapackage directories
are kept for source context but contain `CATKIN_IGNORE` files to avoid pulling
extra packages into the VINS build.
