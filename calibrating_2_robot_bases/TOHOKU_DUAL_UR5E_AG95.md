# Tohoku Ceiling-Mounted Dual UR5e + AG95 Calibration

This is the installation-specific procedure for the two ceiling-mounted UR5e
robots and DH AG95 grippers in `ur_motion_stack`. Read the general
[README.md](README.md) first for the mating assumptions and transform notation.

## Frames used on this installation

The active configuration is [config/robots.yaml](config/robots.yaml):

| Purpose | Frame |
| --- | --- |
| Left robot base | `left/base_link` |
| Right robot base | `right/base_link` |
| Left robot, left pad | `left/ag95/left_pad_surface` |
| Left robot, right pad | `left/ag95/right_pad_surface` |
| Right robot, left pad | `right/ag95/left_pad_surface` |
| Right robot, right pad | `right/ag95/right_pad_surface` |

The pad names use the ceiling-mounted robot's perspective. The interactive
program asks which pad to use on each robot. Keep those selected pads for every
sample; do not alternate pads within one run.

The configuration preserves the current near-accurate placement in comments:

```text
world -> left_base:   xyz [ 0.5, 0.0,  0.000], RPY [pi, 0, 0]
world -> right_base:  xyz [-0.5, 0.0, -0.002], RPY [pi, 0, 0]
left_base -> right_base: xyz [-1.0, 0.0, 0.002], identity rotation
```

The calibration program does not alter this placement or
`robot_mounts.toml`.

## Before touching the robots

- Clear people, tools, and loose objects from both workspaces.
- Verify both PolyScope installations are saved with `Ceiling (180 degrees)`
  mounting and correct payload/CoG values.
- Verify the AG95 pad frames appear at the physical pad surfaces in RViz.
- Make sure no admittance, effort, velocity, or trajectory controller is
  active before entering Freedrive.
- Keep access to both emergency stops throughout the procedure.

## 1. Start both robots and grippers

From the `ur_motion_stack` directory in the activated `ur-motion-stack`
environment:

```bash
ros2 launch dual_ur5e_control start_robots.launch.py \
  launch_admittance:=false \
  launch_crisp_cartesian:=true \
  launch_grippers:=true
```

Wait until both drivers, gripper drivers, and controller managers are ready.

## 2. Start the calibrated TF visualization

In a second terminal with the same environment:

```bash
scripts/run_rviz.sh
```

The default `Key Frames` display should show only the four pad frames, two
AG95 TCPs, two base links, and `world`.

## 3. Start Freedrive for both arms

In a third terminal:

```bash
scripts/run_freedrive.sh both
```

Leave this command running. Pressing `Ctrl+C` stops the Freedrive heartbeat and
deactivates Freedrive on both arms.

## 4. Start the interactive calibration recorder

No manual `tf2_echo`, YAML editing, or quaternion copying is needed. In a
fourth terminal at the `ur_motion_stack` root, run:

```bash
scripts/run_robot_base_calibration.sh
```

The launcher can be invoked from any directory. It finds `ur_motion_stack`,
sets the calibration package's Python path, activates the Pixi/Zenoh settings,
and uses the system Python interpreter that matches ROS Jazzy.

From inside the calibration directory, the shorter equivalent is:

```bash
./run_interactive_calibration.sh
```

Do not replace the launcher with plain `python3`. The running stack uses
`rmw_zenoh_cpp`; a process using another ROS middleware may discover cached
topic names but will not receive TF messages.

The program verifies the middleware, connects directly to
`/dual_ur5e_rviz/tf` and `/dual_ur5e_rviz/tf_static`, and waits for both
selected base-to-pad chains before allowing physical sampling. It then asks:

1. world Mode 1 (`reference base`) or Mode 2 (`midpoint`);
2. which reference base to use if Mode 1 was selected;
3. which pad on the left robot will make contact;
4. which pad on the right robot will make contact; and
5. how many samples to record.

The pad choices use the ceiling-mounted robot's perspective, matching the RViz
frame names. The default is eight samples; use at least five.

Example prompt flow:

```text
Choose the output world mode:
  1) reference base (world coincides with one robot base)
  2) midpoint bimanual world
Selection: 2

Which pad on the LEFT robot will make contact?
  1) left pad
  2) right pad
Selection: 1

Which pad on the RIGHT robot will make contact?
  1) left pad
  2) right pad
Selection: 1

Number of samples [8]: 5
```

Physical up remains configured as `-Z` in the ceiling-mounted left-base frame.
The operator does not need to edit `config/robots.yaml` between Mode 1 and Mode
2 runs.

## 5. Align and record each mating

For every sample, move both arms until the selected pads are:

1. flat and flush across the full surfaces;
2. centered on one another;
3. aligned along both rectangular edges; and
4. stationary and held with minimal contact force.

Do not squeeze or preload the pads; compliance will bias the result. When the
mating is ready and both arms are stationary, press Enter once at this prompt:

```text
Mate and align the pads for sample 1/5, then press Enter to RECORD...
```

The program captures both base-to-pad transforms directly from TF, appends the
sample to a timestamped raw YAML file, and prints the two TF timestamps' skew.
If TF is unavailable, that sample is not counted and the program asks you to
retry. Raw data is saved after every successful sample, so captures survive an
interrupted run.

## 6. Repeat at different configurations

After each capture, separate the pads, move both robots to a substantially
different joint configuration, mate the same selected pads again, and press
Enter at the next prompt.

Vary shoulder, elbow, and wrist configurations rather than making tiny TCP
changes around one pose. Keep every mating accessible and comfortably away
from joint limits and singularities.

After the requested number of samples, the program computes the calibration,
prints the estimated transform and RMS/maximum consistency errors, and writes
both files under `calibration_runs/`:

```text
samples_YYYYMMDD_HHMMSS.yaml
calibration_result_YYYYMMDD_HHMMSS.yaml
```

The run directory is ignored by Git. To select another real, writable
destination, pass its path to the launcher. For example:

```bash
scripts/run_robot_base_calibration.sh \
  --output-dir "$HOME/calibration_data"
```

Omit `--output-dir` to use the recommended default `calibration_runs/` folder.

### Recompute an existing raw capture

Raw sample files are intentionally reusable. After correcting calibration math
or changing only the output world mode, recompute without moving the robots:

```bash
scripts/run_robot_base_calibration.sh \
  --recompute "$HOME/calibration_data/samples_YYYYMMDD_HHMMSS.yaml" \
  --output-dir "$HOME/calibration_data"
```

The program asks only for Mode 1 or Mode 2 (and the reference robot for Mode
1), then writes `calibration_result_recomputed_YYYYMMDD_HHMMSS.yaml`. It never
modifies the raw sample file.

To omit a confirmed outlier without altering the raw capture:

```bash
scripts/run_robot_base_calibration.sh \
  --recompute /path/to/samples.yaml \
  --exclude-label pose_1
```

Repeat `--exclude-label` to omit more than one named sample. Excluded labels
are recorded in the result metadata.

When recording is complete, press `Ctrl+C` in the Freedrive terminal and
confirm both controllers deactivate.

## 7. Review the computed calibration

The important result fields are:

- `left_base_to_right_base`: the invariant calibrated relationship;
- `sample_estimates`: each mating's independent result and error;
- `quality`: RMS and maximum consistency errors; and
- `world_to_left_base` / `world_to_right_base`: placements for the selected
  Mode 1 or Mode 2 world.

There is no universal pass threshold for manual pad mating. Inspect individual
outliers and repeat suspicious samples. Millimetre-scale translation spread or
sub-degree rotation spread may be plausible for a first manual run, but the
acceptable limit must come from the downstream task's accuracy requirement.

## 8. Validate before applying

Keep the existing production transform and compare it with the new
`left_base_to_right_base`. Then perform an independent check:

- mate the unused `right_pad_surface` frames, or
- command both TCPs toward a shared, safely observable fixture point.

Do not use the same recorded samples as the only validation evidence.

## 9. Apply manually after acceptance

The active placement file is:

```text
src/ur5e_motion_stack/config/robot_mounts.toml
```

For the chosen world mode, copy each result's `translation_m` and
`rotation_rpy_rad` into `[robots.left]` and `[robots.right]`. Preserve each
`isaac_prim`. Rebuild and re-source the workspace, restart RViz, and repeat the
independent validation.

Never update the robot controller's factory kinematics calibration YAML with
these values. This procedure estimates the transform between robot bases; it
does not recalibrate either arm's internal kinematic model.
