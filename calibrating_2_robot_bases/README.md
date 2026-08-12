# Full-Pose Two-Robot Base Calibration

This package estimates the rigid transform between two robot bases by mating a
known flat tool surface on one robot with a known flat tool surface on the
other. It is deliberately independent of robot brand, gripper model, ROS, and
the coordinate frame ultimately chosen as `world`.

For the ceiling-mounted dual UR5e and DH AG95 installation in this workspace,
use [TOHOKU_DUAL_UR5E_AG95.md](TOHOKU_DUAL_UR5E_AG95.md). That guide contains
the exact frames, launch commands, TF capture commands, and application steps.
Its interactive workflow is started with `run_interactive_calibration.sh`,
which supplies the local ROS environment and Python package path automatically.

## What the procedure assumes

Each calibration sample must be a complete, known six-degree-of-freedom pad
mating:

- the physical surfaces are flush;
- their defined origins coincide;
- corresponding long and short edges align;
- their surface normals oppose each other; and
- both pad poses are recorded without moving either robot between readings.

Merely touching at a point or somewhere on the two planes is not sufficient.
One ideal full-pose mating is mathematically enough, but repeated samples at
substantially different joint configurations reveal alignment and kinematic
errors. Use at least five; eight to ten are recommended.

## Transform convention and estimator

`T_a_b` is the pose of frame B expressed in frame A and maps coordinates from B
into A. For each sample the calibrator computes:

```text
T_left_base_right_base =
    T_left_base_left_pad
    T_left_pad_right_pad_when_mated
    inverse(T_right_base_right_pad)
```

It averages translation arithmetically and projects the mean rotation back
onto SO(3) with SVD. It then solves one shared translation from all paired
poses using that common rotation, preventing small orientation errors from
being amplified through the robots' reach. It does not average Euler angles.
The output includes the individual estimates and their translation and
geodesic rotation fit residuals, so inconsistency remains visible.

## Requirements

- Python 3.10 or newer
- NumPy
- PyYAML
- pytest, only for running the tests

## Configuration

The configuration identifies the physical frames and the known mating pose:

```yaml
robots:
  left:
    base_frame: left_base
    calibration_pad_frame: left_calibration_pad
  right:
    base_frame: right_base
    calibration_pad_frame: right_calibration_pad

contact:
  left_pad_to_right_pad_when_mated:
    translation_m: [0.0, 0.0, 0.0]
    quaternion_xyzw: [1.0, 0.0, 0.0, 0.0]

sampling:
  minimum_samples: 5

world:
  mode: midpoint
  reference_robot: left
  up_in_left_base: [0.0, 0.0, 1.0]
```

The example mating quaternion is a 180-degree rotation about pad X: pad X
edges align while their +Z surface normals oppose. Change it if your pad-frame
convention is different.

## Recording samples

Copy `config/samples.example.yaml`. Each entry contains simultaneous pad poses,
each expressed in its own robot base:

```yaml
samples:
  - label: pose_1
    left_base_to_left_pad:
      translation_m: [0.4, 0.1, 0.2]
      quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
    right_base_to_right_pad:
      translation_m: [0.3, -0.1, 0.2]
      quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
```

The package intentionally separates pose acquisition from calibration math.
Poses may come from TF, FK, a robot API, or an external tracker, provided they
use the documented frame convention and represent the same stationary mating.

## Run the calibration

From this directory:

```bash
python -m robot_base_calibration.contact_calibrator \
  --config path/to/robots.yaml \
  --samples path/to/samples.yaml \
  --output calibration_result.yaml
```

Omit `--output` to print YAML to standard output. The result contains:

- `left_base_to_right_base`, the calibrated invariant relationship;
- one base-transform estimate and residual per sample;
- RMS and maximum translation/rotation residuals;
- `world_to_left_base` and `world_to_right_base`; and
- quaternion `xyzw` and fixed-axis RPY radians for each output pose.

## World-frame modes

World selection happens after calibration and does not change the estimated
base-to-base relationship.

### Mode 1: `reference_base`

`world` coincides with either the left or right base:

```yaml
world:
  mode: reference_base
  reference_robot: left
```

### Mode 2: `midpoint`

`world` lies halfway between the base origins. +X points from the left origin
toward the right origin after projection perpendicular to the configured up
direction, and +Z follows `up_in_left_base`:

```yaml
world:
  mode: midpoint
  up_in_left_base: [0.0, 0.0, 1.0]
```

## Python API

```python
from robot_base_calibration import ContactCalibrator, pose_to_transform

calibrator = ContactCalibrator(mating_transform, minimum_samples=5)
calibrator.add_sample(T_left_base_left_pad, T_right_base_right_pad)
result = calibrator.calibrate()

world = calibrator.world_transforms(
    result["T_left_base_right_base"],
    mode="midpoint",
    up_in_left_base=(0.0, 0.0, 1.0),
)
```

## Safety and application policy

The calibrator only writes the requested result YAML. It never modifies robot
descriptions, controller calibration, or production world transforms. Review
sample residuals, validate the result independently, and apply it manually.

Do not commit machine credentials or private device identifiers.
