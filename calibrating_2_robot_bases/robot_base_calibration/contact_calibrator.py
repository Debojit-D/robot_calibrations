"""Stateful full-pose pad-mating base calibrator and offline CLI."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import yaml

from .calibration_math import (
    compute_base_transform,
    make_midpoint_world,
    make_reference_base_world,
    paired_transform_residuals,
    pose_to_transform,
    solve_paired_transforms,
    transform_to_pose,
    validate_transform,
)


@dataclass(frozen=True)
class CalibrationSample:
    """One simultaneous pair of pad poses expressed in their own bases."""

    label: str
    t_left_base_left_pad: np.ndarray
    t_right_base_right_pad: np.ndarray


class ContactCalibrator:
    """Accumulate flush pad matings and estimate the inter-base transform."""

    def __init__(
        self,
        mating_transform: Sequence[Sequence[float]],
        *,
        minimum_samples: int = 1,
    ) -> None:
        if minimum_samples < 1:
            raise ValueError("minimum_samples must be at least one")
        self.mating_transform = validate_transform(mating_transform, "mating_transform")
        self.minimum_samples = minimum_samples
        self.samples: list[CalibrationSample] = []

    def add_sample(
        self,
        t_left_base_left_pad: Sequence[Sequence[float]],
        t_right_base_right_pad: Sequence[Sequence[float]],
        *,
        label: str | None = None,
    ) -> None:
        """Add pad poses captured while pad centers and edges are fully aligned."""
        sample_number = len(self.samples) + 1
        self.samples.append(
            CalibrationSample(
                label=label or f"sample_{sample_number}",
                t_left_base_left_pad=validate_transform(
                    t_left_base_left_pad, "T_left_base_left_pad"
                ),
                t_right_base_right_pad=validate_transform(
                    t_right_base_right_pad, "T_right_base_right_pad"
                ),
            )
        )

    def sample_estimates(self) -> list[np.ndarray]:
        """Return one independent ``T_left_base_right_base`` per mating."""
        return [
            compute_base_transform(
                sample.t_left_base_left_pad,
                sample.t_right_base_right_pad,
                self.mating_transform,
            )
            for sample in self.samples
        ]

    def calibrate(self) -> dict[str, object]:
        """Average all samples and return the estimate plus quality metrics."""
        if len(self.samples) < self.minimum_samples:
            raise RuntimeError(
                f"need at least {self.minimum_samples} samples; have {len(self.samples)}"
            )
        estimates = self.sample_estimates()
        target_poses = [
            sample.t_left_base_left_pad @ self.mating_transform
            for sample in self.samples
        ]
        source_poses = [
            sample.t_right_base_right_pad for sample in self.samples
        ]
        estimate = solve_paired_transforms(target_poses, source_poses)
        return {
            "sample_count": len(estimates),
            "sample_labels": [sample.label for sample in self.samples],
            "T_left_base_right_base": estimate,
            "sample_estimates": estimates,
            "residuals": paired_transform_residuals(
                target_poses, source_poses, estimate
            ),
        }

    @staticmethod
    def world_transforms(
        t_left_base_right_base: Sequence[Sequence[float]],
        *,
        mode: int | str,
        reference_robot: str = "left",
        up_in_left_base: Sequence[float] = (0.0, 0.0, 1.0),
    ) -> dict[str, np.ndarray]:
        """Apply Mode 1 (reference base) or Mode 2 (midpoint world)."""
        normalized = str(mode).lower().replace("-", "_")
        if normalized in {"1", "mode_1", "reference_base"}:
            return make_reference_base_world(t_left_base_right_base, reference_robot)
        if normalized in {"2", "mode_2", "midpoint"}:
            return make_midpoint_world(t_left_base_right_base, up_in_left_base)
        raise ValueError("mode must be 1/reference_base or 2/midpoint")


def _load_yaml(path: Path) -> Mapping[str, object]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"could not load {path}: {error}") from error
    if not isinstance(document, Mapping):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def serialize_calibration_result(
    result: Mapping[str, object],
    world: Mapping[str, np.ndarray],
    config_path: Path,
) -> dict[str, object]:
    residuals = dict(result["residuals"])
    residuals["rotation_rms_deg"] = float(
        np.degrees(residuals["rotation_rms_rad"])
    )
    residuals["rotation_max_deg"] = float(
        np.degrees(residuals["rotation_max_rad"])
    )
    sample_estimates = []
    for index, (label, estimate) in enumerate(
        zip(result["sample_labels"], result["sample_estimates"])
    ):
        sample_estimates.append(
            {
                "label": label,
                "left_base_to_right_base": transform_to_pose(estimate),
                "translation_error_m": residuals["translation_error_m"][index],
                "rotation_error_deg": float(
                    np.degrees(residuals["rotation_error_rad"][index])
                ),
            }
        )
    return {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_config": str(config_path),
        "sample_count": result["sample_count"],
        "left_base_to_right_base": transform_to_pose(result["T_left_base_right_base"]),
        "sample_estimates": sample_estimates,
        "world_to_left_base": transform_to_pose(world["T_world_left_base"]),
        "world_to_right_base": transform_to_pose(world["T_world_right_base"]),
        "quality": residuals,
    }


def run_from_files(config_path: Path, samples_path: Path) -> dict[str, object]:
    """Run a complete offline calibration from the documented YAML schema."""
    config = _load_yaml(config_path)
    samples_document = _load_yaml(samples_path)
    contact = config["contact"]
    sampling = config.get("sampling", {})
    calibrator = ContactCalibrator(
        pose_to_transform(contact["left_pad_to_right_pad_when_mated"]),
        minimum_samples=int(sampling.get("minimum_samples", 1)),
    )
    samples = samples_document.get("samples")
    if not isinstance(samples, list):
        raise ValueError("samples file must contain a 'samples' list")
    for index, sample in enumerate(samples, start=1):
        calibrator.add_sample(
            pose_to_transform(sample["left_base_to_left_pad"]),
            pose_to_transform(sample["right_base_to_right_pad"]),
            label=str(sample.get("label", f"sample_{index}")),
        )
    result = calibrator.calibrate()
    world_config = config.get("world", {})
    world = calibrator.world_transforms(
        result["T_left_base_right_base"],
        mode=world_config.get("mode", "midpoint"),
        reference_robot=str(world_config.get("reference_robot", "left")),
        up_in_left_base=world_config.get("up_in_left_base", (0.0, 0.0, 1.0)),
    )
    return serialize_calibration_result(result, world, config_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output", type=Path, help="write result YAML; stdout otherwise")
    arguments = parser.parse_args()
    output = yaml.safe_dump(
        run_from_files(arguments.config, arguments.samples),
        sort_keys=False,
    )
    if arguments.output is None:
        print(output, end="")
    else:
        arguments.output.write_text(output, encoding="utf-8")
        print(f"Wrote calibration result to {arguments.output}")


if __name__ == "__main__":
    main()
    paired_transform_residuals,
    solve_paired_transforms,
