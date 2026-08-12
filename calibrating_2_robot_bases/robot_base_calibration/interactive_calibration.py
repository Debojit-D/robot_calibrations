"""Interactively capture dual-robot pad matings from ROS TF and calibrate."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
from typing import Mapping

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .calibration_math import pose_to_transform
from .contact_calibrator import (
    ContactCalibrator,
    serialize_calibration_result,
)


TF_TOPIC = "/dual_ur5e_rviz/tf"
TF_STATIC_TOPIC = "/dual_ur5e_rviz/tf_static"
PAD_FRAMES = {
    "left": {
        "1": "left/ag95/left_pad_surface",
        "2": "left/ag95/right_pad_surface",
    },
    "right": {
        "1": "right/ag95/left_pad_surface",
        "2": "right/ag95/right_pad_surface",
    },
}
BASE_FRAMES = {"left": "left/base_link", "right": "right/base_link"}
REQUIRED_RMW = "rmw_zenoh_cpp"
ZENOH_CONFIG_VARIABLES = (
    "ZENOH_SESSION_CONFIG_URI",
    "ZENOH_ROUTER_CONFIG_URI",
)


def _workspace_root() -> Path:
    """Find the enclosing ur_motion_stack root containing pixi.toml."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "pixi.toml").is_file():
            return parent
    raise RuntimeError("could not locate the ur_motion_stack root")


def _normalize_zenoh_config_paths() -> None:
    """Make Pixi's workspace-relative Zenoh paths valid from this subdirectory."""
    root = _workspace_root()
    for variable in ZENOH_CONFIG_VARIABLES:
        value = os.environ.get(variable)
        if not value or "://" in value:
            continue
        path = Path(value)
        if not path.is_absolute():
            os.environ[variable] = str((root / path).resolve())


def _load_yaml(path: Path) -> Mapping[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def _ask_choice(prompt: str, choices: Mapping[str, str]) -> str:
    """Ask until the operator enters one of the displayed keys."""
    while True:
        print(prompt)
        for key, description in choices.items():
            print(f"  {key}) {description}")
        answer = input("Selection: ").strip()
        if answer in choices:
            return answer
        print(f"Please enter one of: {', '.join(choices)}\n")


def _ask_sample_count(default: int, minimum: int) -> int:
    """Ask for a sample count satisfying the configured minimum."""
    while True:
        answer = input(f"Number of samples [{default}]: ").strip()
        if not answer:
            return default
        try:
            count = int(answer)
        except ValueError:
            count = 0
        if count >= minimum:
            return count
        print(f"Enter an integer of at least {minimum}.\n")


def _transform_pose(message) -> dict[str, list[float]]:
    transform = message.transform
    return {
        "translation_m": [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        "quaternion_xyzw": [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    }


def _stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _write_yaml(path: Path, document: Mapping[str, object]) -> None:
    """Persist captures after every sample so an interrupted run is recoverable."""
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


class PadPoseCapture(Node):
    """Read the two selected base-to-pad transforms from the RViz TF tree."""

    def __init__(self) -> None:
        super().__init__("interactive_robot_base_calibration")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)

    def capture(self, left_pad: str, right_pad: str) -> tuple[object, object]:
        """Return the latest transforms for one stationary physical mating."""
        timeout = Duration(seconds=3.0)
        left = self.buffer.lookup_transform(
            BASE_FRAMES["left"], left_pad, Time(), timeout=timeout
        )
        right = self.buffer.lookup_transform(
            BASE_FRAMES["right"], right_pad, Time(), timeout=timeout
        )
        return left, right

    def wait_until_ready(self, left_pad: str, right_pad: str) -> None:
        """Fail before physical mating if either selected TF chain is absent."""
        timeout = Duration(seconds=10.0)
        missing = []
        for base, pad in (
            (BASE_FRAMES["left"], left_pad),
            (BASE_FRAMES["right"], right_pad),
        ):
            if not self.buffer.can_transform(base, pad, Time(), timeout=timeout):
                missing.append(f"{base} -> {pad}")
        if missing:
            joined = "\n  - ".join(missing)
            raise RuntimeError(
                "required calibration TF chains are unavailable:\n"
                f"  - {joined}\n"
                "Confirm scripts/run_rviz.sh is running in the same "
                "ur-motion-stack environment."
            )

    def close(self) -> None:
        self.listener.unregister()
        self.destroy_node()


def _world_selection() -> dict[str, str]:
    mode = _ask_choice(
        "Choose the output world mode:",
        {
            "1": "reference base (world coincides with one robot base)",
            "2": "midpoint bimanual world",
        },
    )
    reference_robot = "left"
    if mode == "1":
        reference_robot = _ask_choice(
            "Which robot base should be world?", {"left": "left", "right": "right"}
        )
    return {"mode": mode, "reference_robot": reference_robot}


def _configuration(config: Mapping[str, object]) -> dict[str, object]:
    selection = _world_selection()
    left_pad_key = _ask_choice(
        "Which pad on the LEFT robot will make contact?",
        {"1": "left pad", "2": "right pad"},
    )
    right_pad_key = _ask_choice(
        "Which pad on the RIGHT robot will make contact?",
        {"1": "left pad", "2": "right pad"},
    )
    sampling = config.get("sampling", {})
    minimum_count = int(sampling.get("minimum_samples", 5))
    default_count = int(sampling.get("recommended_samples", 8))
    selection.update({
        "left_pad_frame": PAD_FRAMES["left"][left_pad_key],
        "right_pad_frame": PAD_FRAMES["right"][right_pad_key],
        "sample_count": _ask_sample_count(default_count, minimum_count),
    })
    return selection


def _print_calibration_summary(result: Mapping[str, object]) -> None:
    quality = result["quality"]
    estimate = result["left_base_to_right_base"]
    print("\nCalibration quality")
    print(f"  Translation RMS: {quality['translation_rms_m'] * 1000.0:.3f} mm")
    print(f"  Translation max: {quality['translation_max_m'] * 1000.0:.3f} mm")
    print(f"  Rotation RMS:    {quality['rotation_rms_deg']:.4f} deg")
    print(f"  Rotation max:    {quality['rotation_max_deg']:.4f} deg")
    print("\nEstimated left_base -> right_base")
    print(f"  translation_m:    {estimate['translation_m']}")
    print(f"  quaternion_xyzw:  {estimate['quaternion_xyzw']}")
    print(f"  rotation_rpy_rad: {estimate['rotation_rpy_rad']}")


def run_interactive(config_path: Path, output_directory: Path) -> tuple[Path, Path]:
    """Prompt, capture, save raw data, calibrate, and return both output paths."""
    config = _load_yaml(config_path)
    selection = _configuration(config)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_directory.mkdir(parents=True, exist_ok=True)
    samples_path = output_directory / f"samples_{timestamp}.yaml"
    result_path = output_directory / f"calibration_result_{timestamp}.yaml"
    samples_document = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "left_pad_frame": selection["left_pad_frame"],
        "right_pad_frame": selection["right_pad_frame"],
        "samples": [],
    }
    _write_yaml(samples_path, samples_document)

    contact = config["contact"]
    calibrator = ContactCalibrator(
        pose_to_transform(contact["left_pad_to_right_pad_when_mated"]),
        minimum_samples=int(selection["sample_count"]),
    )

    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            f"/tf:={TF_TOPIC}",
            "-r",
            f"/tf_static:={TF_STATIC_TOPIC}",
        ]
    )
    node = PadPoseCapture()
    try:
        print("Waiting for both selected TF chains...")
        node.wait_until_ready(
            str(selection["left_pad_frame"]),
            str(selection["right_pad_frame"]),
        )
        print("\nInteractive calibration is ready.")
        print(f"  Left:  {selection['left_pad_frame']}")
        print(f"  Right: {selection['right_pad_frame']}")
        print(f"  Raw captures: {samples_path}")
        for index in range(1, int(selection["sample_count"]) + 1):
            while True:
                input(
                    f"\nMate and align the pads for sample {index}/"
                    f"{selection['sample_count']}, then press Enter to RECORD..."
                )
                try:
                    left_message, right_message = node.capture(
                        str(selection["left_pad_frame"]),
                        str(selection["right_pad_frame"]),
                    )
                except TransformException as error:
                    print(f"TF capture failed: {error}\nCorrect the issue and retry.")
                    continue
                left_pose = _transform_pose(left_message)
                right_pose = _transform_pose(right_message)
                skew = abs(_stamp_seconds(left_message) - _stamp_seconds(right_message))
                label = f"pose_{index}"
                calibrator.add_sample(
                    pose_to_transform(left_pose),
                    pose_to_transform(right_pose),
                    label=label,
                )
                samples_document["samples"].append(
                    {
                        "label": label,
                        "captured_utc": datetime.now(timezone.utc).isoformat(),
                        "tf_timestamp_skew_s": skew,
                        "left_base_to_left_pad": left_pose,
                        "right_base_to_right_pad": right_pose,
                    }
                )
                _write_yaml(samples_path, samples_document)
                print(f"Recorded {label}; TF timestamp skew = {skew:.6f} s")
                break
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()

    calibration = calibrator.calibrate()
    world_config = config.get("world", {})
    world = calibrator.world_transforms(
        calibration["T_left_base_right_base"],
        mode=selection["mode"],
        reference_robot=str(selection["reference_robot"]),
        up_in_left_base=world_config.get("up_in_left_base", (0.0, 0.0, -1.0)),
    )
    result = serialize_calibration_result(calibration, world, config_path)
    result["interactive_selection"] = selection
    result["raw_samples_file"] = str(samples_path)
    _write_yaml(result_path, result)
    _print_calibration_summary(result)
    return samples_path, result_path


def recompute_samples(
    config_path: Path,
    samples_path: Path,
    output_directory: Path,
    excluded_labels: set[str] | None = None,
) -> Path:
    """Recompute a result from preserved raw samples without contacting ROS."""
    config = _load_yaml(config_path)
    document = _load_yaml(samples_path)
    samples = document.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"{samples_path} must contain a nonempty samples list")
    excluded_labels = excluded_labels or set()
    available_labels = {
        str(sample.get("label", f"pose_{index}"))
        for index, sample in enumerate(samples, start=1)
    }
    unknown_labels = excluded_labels - available_labels
    if unknown_labels:
        raise ValueError(
            "cannot exclude unknown sample label(s): "
            + ", ".join(sorted(unknown_labels))
        )
    samples = [
        sample
        for index, sample in enumerate(samples, start=1)
        if str(sample.get("label", f"pose_{index}")) not in excluded_labels
    ]
    sampling = config.get("sampling", {})
    calibrator = ContactCalibrator(
        pose_to_transform(
            config["contact"]["left_pad_to_right_pad_when_mated"]
        ),
        minimum_samples=int(sampling.get("minimum_samples", 1)),
    )
    for index, sample in enumerate(samples, start=1):
        calibrator.add_sample(
            pose_to_transform(sample["left_base_to_left_pad"]),
            pose_to_transform(sample["right_base_to_right_pad"]),
            label=str(sample.get("label", f"pose_{index}")),
        )
    selection = _world_selection()
    selection.update(
        {
            "left_pad_frame": document.get("left_pad_frame"),
            "right_pad_frame": document.get("right_pad_frame"),
            "sample_count": len(samples),
            "recomputed": True,
            "excluded_labels": sorted(excluded_labels),
        }
    )
    calibration = calibrator.calibrate()
    world_config = config.get("world", {})
    world = calibrator.world_transforms(
        calibration["T_left_base_right_base"],
        mode=selection["mode"],
        reference_robot=selection["reference_robot"],
        up_in_left_base=world_config.get("up_in_left_base", (0.0, 0.0, -1.0)),
    )
    result = serialize_calibration_result(calibration, world, config_path)
    result["interactive_selection"] = selection
    result["raw_samples_file"] = str(samples_path.resolve())
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_path = output_directory / f"calibration_result_recomputed_{timestamp}.yaml"
    _write_yaml(result_path, result)
    _print_calibration_summary(result)
    return result_path


def main() -> None:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=default_root / "config" / "robots.yaml"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=default_root / "calibration_runs"
    )
    parser.add_argument(
        "--recompute",
        type=Path,
        metavar="RAW_SAMPLES_YAML",
        help="recompute from saved raw samples without recording new TF data",
    )
    parser.add_argument(
        "--exclude-label",
        action="append",
        default=[],
        metavar="SAMPLE_LABEL",
        help="exclude a named sample during --recompute; may be repeated",
    )
    arguments = parser.parse_args()
    if arguments.recompute is not None:
        try:
            result_path = recompute_samples(
                arguments.config,
                arguments.recompute,
                arguments.output_dir,
                set(arguments.exclude_label),
            )
        except (KeyboardInterrupt, EOFError):
            print("\nRecomputation cancelled.")
            return
        print("\nRecomputation complete.")
        print(f"Raw samples: {arguments.recompute}")
        print(f"Result:      {result_path}")
        return
    active_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if active_rmw != REQUIRED_RMW:
        parser.error(
            f"this workspace requires RMW_IMPLEMENTATION={REQUIRED_RMW}, but "
            f"the current value is {active_rmw or 'unset'}. From this "
            "calibration directory run: pixi run /usr/bin/python3 -m "
            "robot_base_calibration.interactive_calibration, or first activate "
            "the ur-motion-stack environment."
        )
    _normalize_zenoh_config_paths()
    try:
        samples_path, result_path = run_interactive(
            arguments.config, arguments.output_dir
        )
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration cancelled. Any recorded raw samples were preserved.")
        return
    print("\nCalibration complete.")
    print(f"Raw samples: {samples_path}")
    print(f"Result:      {result_path}")


if __name__ == "__main__":
    main()
