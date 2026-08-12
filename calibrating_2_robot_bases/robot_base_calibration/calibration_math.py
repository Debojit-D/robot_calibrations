"""Robot-independent rigid-transform math for pad-mating calibration.

The notation ``T_a_b`` means the pose of frame B expressed in frame A.  A
homogeneous transform therefore maps coordinates from B into A.
"""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Sequence

import numpy as np


_EPSILON = 1.0e-12


def _vector(value: Sequence[float], size: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return vector


def validate_transform(transform: Sequence[Sequence[float]], name: str = "transform") -> np.ndarray:
    """Return a checked copy of a homogeneous rigid transform."""
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-9):
        raise ValueError(f"{name} must have homogeneous bottom row [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-7):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-7):
        raise ValueError(f"{name} rotation determinant must be +1")
    return matrix.copy()


def quaternion_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Convert a normalized-or-normalizable XYZW quaternion to a matrix."""
    x, y, z, w = _vector(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < _EPSILON:
        raise ValueError("quaternion_xyzw must be nonzero")
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(rotation: Sequence[Sequence[float]]) -> np.ndarray:
    """Convert a proper rotation matrix to a canonical XYZW quaternion."""
    candidate = np.eye(4)
    candidate[:3, :3] = np.asarray(rotation, dtype=float)
    matrix = validate_transform(candidate, "rotation")[:3, :3]

    # Eigenvector form of the Bar-Itzhack conversion.  It remains stable near
    # 180 degrees, where trace-based formulas are numerically awkward.
    rxx, rxy, rxz = matrix[0]
    ryx, ryy, ryz = matrix[1]
    rzx, rzy, rzz = matrix[2]
    k_matrix = np.array(
        [
            [rxx - ryy - rzz, rxy + ryx, rxz + rzx, ryz - rzy],
            [rxy + ryx, ryy - rxx - rzz, ryz + rzy, rzx - rxz],
            [rxz + rzx, ryz + rzy, rzz - rxx - ryy, rxy - ryx],
            [ryz - rzy, rzx - rxz, rxy - ryx, rxx + ryy + rzz],
        ],
        dtype=float,
    ) / 3.0
    quaternion = np.linalg.eigh(k_matrix)[1][:, -1]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def matrix_to_rpy(rotation: Sequence[Sequence[float]]) -> np.ndarray:
    """Convert a matrix to fixed-axis roll, pitch, yaw radians (Rz Ry Rx)."""
    candidate = np.eye(4)
    candidate[:3, :3] = np.asarray(rotation, dtype=float)
    matrix = validate_transform(candidate, "rotation")[:3, :3]
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        # At gimbal lock, choose yaw=0 and put the observable combined angle
        # into roll. This remains a valid representation of the same rotation.
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return np.array((roll, pitch, yaw))


def pose_to_transform(pose: Mapping[str, Sequence[float]]) -> np.ndarray:
    """Build ``T`` from ``translation_m`` and ``quaternion_xyzw`` fields."""
    translation = _vector(pose["translation_m"], 3, "translation_m")
    transform = np.eye(4)
    transform[:3, :3] = quaternion_xyzw_to_matrix(pose["quaternion_xyzw"])
    transform[:3, 3] = translation
    return transform


def transform_to_pose(transform: Sequence[Sequence[float]]) -> dict[str, list[float]]:
    """Serialize ``T`` using metres and an XYZW quaternion."""
    matrix = validate_transform(transform)
    return {
        "translation_m": matrix[:3, 3].tolist(),
        "quaternion_xyzw": matrix_to_quaternion_xyzw(matrix[:3, :3]).tolist(),
        "rotation_rpy_rad": matrix_to_rpy(matrix[:3, :3]).tolist(),
    }


def invert_transform(transform: Sequence[Sequence[float]]) -> np.ndarray:
    """Invert a rigid transform without a general matrix inverse."""
    matrix = validate_transform(transform)
    inverse = np.eye(4)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return inverse


def compute_base_transform(
    t_left_base_left_pad: Sequence[Sequence[float]],
    t_right_base_right_pad: Sequence[Sequence[float]],
    t_left_pad_right_pad_mated: Sequence[Sequence[float]],
) -> np.ndarray:
    """Estimate ``T_left_base_right_base`` from one fully mated pad pose.

    The pad origins must coincide on their physical contact surfaces, and the
    mating transform must encode the known edge alignment and opposed normals.
    """
    t_bl_pl = validate_transform(t_left_base_left_pad, "T_left_base_left_pad")
    t_br_pr = validate_transform(t_right_base_right_pad, "T_right_base_right_pad")
    t_pl_pr = validate_transform(t_left_pad_right_pad_mated, "T_left_pad_right_pad_mated")
    return t_bl_pl @ t_pl_pr @ invert_transform(t_br_pr)


def average_transforms(transforms: Iterable[Sequence[Sequence[float]]]) -> np.ndarray:
    """Compute a chordal SO(3) mean and arithmetic translation mean."""
    matrices = [validate_transform(item) for item in transforms]
    if not matrices:
        raise ValueError("at least one transform is required")
    mean_rotation = np.mean([item[:3, :3] for item in matrices], axis=0)
    u_matrix, _, vt_matrix = np.linalg.svd(mean_rotation)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u_matrix @ vt_matrix)
    result = np.eye(4)
    result[:3, :3] = u_matrix @ correction @ vt_matrix
    result[:3, 3] = np.mean([item[:3, 3] for item in matrices], axis=0)
    return result


def solve_paired_transforms(
    target_poses: Iterable[Sequence[Sequence[float]]],
    source_poses: Iterable[Sequence[Sequence[float]]],
) -> np.ndarray:
    """Least-squares solve for one transform mapping every source to target.

    Rotation is solved first from all relative rotations. Translation is then
    solved with that single shared rotation, avoiding lever-arm amplification
    from the noisy per-sample rotations.
    """
    targets = [validate_transform(item, "target_pose") for item in target_poses]
    sources = [validate_transform(item, "source_pose") for item in source_poses]
    if not targets or len(targets) != len(sources):
        raise ValueError("target_poses and source_poses must have equal nonzero length")
    relative_rotations = []
    for target, source in zip(targets, sources):
        relative = np.eye(4)
        relative[:3, :3] = target[:3, :3] @ source[:3, :3].T
        relative_rotations.append(relative)
    rotation = average_transforms(relative_rotations)[:3, :3]
    translation = np.mean(
        [target[:3, 3] - rotation @ source[:3, 3] for target, source in zip(targets, sources)],
        axis=0,
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def rotation_distance_rad(left: np.ndarray, right: np.ndarray) -> float:
    """Return the geodesic angular distance between two rotations."""
    relative = left.T @ right
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def paired_transform_residuals(
    target_poses: Iterable[Sequence[Sequence[float]]],
    source_poses: Iterable[Sequence[Sequence[float]]],
    estimate: Sequence[Sequence[float]],
) -> dict[str, object]:
    """Measure pose-fit errors for ``target = estimate @ source``."""
    targets = [validate_transform(item, "target_pose") for item in target_poses]
    sources = [validate_transform(item, "source_pose") for item in source_poses]
    if not targets or len(targets) != len(sources):
        raise ValueError("target_poses and source_poses must have equal nonzero length")
    fitted = validate_transform(estimate, "estimate")
    translation_m = []
    rotation_rad = []
    for target, source in zip(targets, sources):
        prediction = fitted @ source
        translation_m.append(float(np.linalg.norm(prediction[:3, 3] - target[:3, 3])))
        rotation_rad.append(rotation_distance_rad(target[:3, :3], prediction[:3, :3]))
    translation = np.asarray(translation_m)
    rotation = np.asarray(rotation_rad)
    return {
        "translation_error_m": translation_m,
        "rotation_error_rad": rotation_rad,
        "translation_rms_m": float(np.sqrt(np.mean(translation**2))),
        "translation_max_m": float(np.max(translation)),
        "rotation_rms_rad": float(np.sqrt(np.mean(rotation**2))),
        "rotation_max_rad": float(np.max(rotation)),
    }


def transform_residuals(
    transforms: Iterable[Sequence[Sequence[float]]],
    estimate: Sequence[Sequence[float]],
) -> dict[str, object]:
    """Measure every estimate's translation and rotation distance from a mean."""
    reference = validate_transform(estimate, "estimate")
    matrices = [validate_transform(item) for item in transforms]
    if not matrices:
        raise ValueError("at least one transform is required")
    translation_m = np.array(
        [np.linalg.norm(item[:3, 3] - reference[:3, 3]) for item in matrices]
    )
    rotation_rad = np.array(
        [rotation_distance_rad(reference[:3, :3], item[:3, :3]) for item in matrices]
    )
    return {
        "translation_error_m": translation_m.tolist(),
        "rotation_error_rad": rotation_rad.tolist(),
        "translation_rms_m": float(np.sqrt(np.mean(translation_m**2))),
        "translation_max_m": float(np.max(translation_m)),
        "rotation_rms_rad": float(np.sqrt(np.mean(rotation_rad**2))),
        "rotation_max_rad": float(np.max(rotation_rad)),
    }


def make_reference_base_world(
    t_left_base_right_base: Sequence[Sequence[float]],
    reference_robot: str = "left",
) -> dict[str, np.ndarray]:
    """Mode 1: place ``world`` exactly on the selected robot base."""
    t_bl_br = validate_transform(t_left_base_right_base, "T_left_base_right_base")
    if reference_robot == "left":
        return {"T_world_left_base": np.eye(4), "T_world_right_base": t_bl_br}
    if reference_robot == "right":
        return {
            "T_world_left_base": invert_transform(t_bl_br),
            "T_world_right_base": np.eye(4),
        }
    raise ValueError("reference_robot must be 'left' or 'right'")


def make_midpoint_world(
    t_left_base_right_base: Sequence[Sequence[float]],
    up_in_left_base: Sequence[float],
) -> dict[str, np.ndarray]:
    """Mode 2: put world midway between bases, with +X pointing left-to-right.

    ``up_in_left_base`` disambiguates roll for floor- or ceiling-mounted robots.
    The base-to-base direction is projected perpendicular to this up vector.
    """
    t_bl_br = validate_transform(t_left_base_right_base, "T_left_base_right_base")
    up = _vector(up_in_left_base, 3, "up_in_left_base")
    up_norm = np.linalg.norm(up)
    if up_norm < _EPSILON:
        raise ValueError("up_in_left_base must be nonzero")
    z_axis = up / up_norm
    baseline = t_bl_br[:3, 3]
    x_axis = baseline - z_axis * float(np.dot(baseline, z_axis))
    x_norm = np.linalg.norm(x_axis)
    if x_norm < _EPSILON:
        raise ValueError("base separation must not be parallel to up_in_left_base")
    x_axis /= x_norm
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)

    # Pose of midpoint world W in the left base: columns are W axes in B_L.
    t_bl_w = np.eye(4)
    t_bl_w[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    t_bl_w[:3, 3] = 0.5 * baseline
    t_w_bl = invert_transform(t_bl_w)
    return {
        "T_world_left_base": t_w_bl,
        "T_world_right_base": t_w_bl @ t_bl_br,
    }
