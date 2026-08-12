# Robot Calibrations

Robot-specific calibration procedures, scripts, and generated calibration data
used by the Touch2Screw stack live in this repository.

## Layout

- `calibrating_2_robot_bases/`: calibration work for estimating the relative
  transforms between two robot bases. Its `README.md` is robot-agnostic;
  `TOHOKU_DUAL_UR5E_AG95.md` documents the local ceiling-mounted setup.

This repository is consumed by Touch2Screw as a Git submodule. Commit and push
changes from this repository independently before updating the submodule
reference in Touch2Screw.
