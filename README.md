# Robot Calibrations

Robot-specific calibration procedures, scripts, and generated calibration data
used by the Touch2Screw stack live in this repository.

## Layout

- `calibrating_2_robot_bases/`: calibration work for estimating the relative
  transforms between the two robot bases.

This repository is consumed by Touch2Screw as a Git submodule. Commit and push
changes from this repository independently before updating the submodule
reference in Touch2Screw.
