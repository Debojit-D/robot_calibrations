"""Full-pose pad-mating calibration for a pair of robot bases."""

from .calibration_math import (
    average_transforms,
    compute_base_transform,
    invert_transform,
    matrix_to_rpy,
    make_midpoint_world,
    make_reference_base_world,
    paired_transform_residuals,
    pose_to_transform,
    solve_paired_transforms,
    transform_residuals,
    transform_to_pose,
)
__all__ = [
    "ContactCalibrator",
    "average_transforms",
    "compute_base_transform",
    "invert_transform",
    "matrix_to_rpy",
    "make_midpoint_world",
    "make_reference_base_world",
    "paired_transform_residuals",
    "pose_to_transform",
    "solve_paired_transforms",
    "transform_residuals",
    "transform_to_pose",
    "serialize_calibration_result",
]


def __getattr__(name: str):
    """Load the CLI-bearing module lazily so ``python -m`` stays warning-free."""
    if name in {"ContactCalibrator", "serialize_calibration_result"}:
        from .contact_calibrator import (
            ContactCalibrator,
            serialize_calibration_result,
        )

        return {
            "ContactCalibrator": ContactCalibrator,
            "serialize_calibration_result": serialize_calibration_result,
        }[name]
    raise AttributeError(name)
