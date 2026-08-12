import math
import os
from pathlib import Path
from unittest.mock import patch

import numpy as np

from robot_base_calibration import (
    ContactCalibrator,
    compute_base_transform,
    make_midpoint_world,
    pose_to_transform,
    solve_paired_transforms,
    transform_to_pose,
)
from robot_base_calibration.interactive_calibration import (
    _configuration,
    _normalize_zenoh_config_paths,
)


def transform(translation, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return pose_to_transform(
        {"translation_m": translation, "quaternion_xyzw": quaternion}
    )


def test_one_mating_recovers_base_transform():
    t_left_base_right_base = transform((-1.0, 0.02, 0.003))
    mating = transform((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    t_left_base_left_pad = transform((-0.35, 0.1, 0.4))
    # Rearranged from T_BL_BR = T_BL_PL T_PL_PR inv(T_BR_PR).
    t_right_base_right_pad = (
        np.linalg.inv(t_left_base_right_base) @ t_left_base_left_pad @ mating
    )
    recovered = compute_base_transform(
        t_left_base_left_pad, t_right_base_right_pad, mating
    )
    np.testing.assert_allclose(recovered, t_left_base_right_base, atol=1.0e-10)


def test_calibrator_averages_translation_and_reports_spread():
    mating = np.eye(4)
    calibrator = ContactCalibrator(mating, minimum_samples=2)
    calibrator.add_sample(transform((1.0, 0.0, 0.0)), np.eye(4))
    calibrator.add_sample(transform((1.002, 0.0, 0.0)), np.eye(4))
    result = calibrator.calibrate()
    np.testing.assert_allclose(
        result["T_left_base_right_base"][:3, 3], (1.001, 0.0, 0.0)
    )
    assert math.isclose(result["residuals"]["translation_rms_m"], 0.001)


def test_paired_solver_uses_one_shared_rotation_for_translation():
    true_base = transform((-1.0, 0.02, 0.003))
    sources = [transform((0.5, offset, 0.6)) for offset in (-0.2, 0.0, 0.2)]
    targets = [true_base @ source for source in sources]
    estimate = solve_paired_transforms(targets, sources)
    np.testing.assert_allclose(estimate, true_base, atol=1.0e-12)


def test_mode_1_supports_either_reference_base():
    relative = transform((2.0, 0.0, 0.0))
    left_reference = ContactCalibrator.world_transforms(relative, mode=1)
    np.testing.assert_allclose(left_reference["T_world_left_base"], np.eye(4))
    np.testing.assert_allclose(left_reference["T_world_right_base"], relative)

    right_reference = ContactCalibrator.world_transforms(
        relative, mode="reference_base", reference_robot="right"
    )
    np.testing.assert_allclose(right_reference["T_world_right_base"], np.eye(4))
    np.testing.assert_allclose(right_reference["T_world_left_base"][:3, 3], (-2, 0, 0))


def test_mode_2_places_world_at_midpoint_with_requested_up():
    relative = transform((-1.0, 0.0, 0.002))
    world = make_midpoint_world(relative, up_in_left_base=(0.0, 0.0, -1.0))
    left_position = world["T_world_left_base"][:3, 3]
    right_position = world["T_world_right_base"][:3, 3]
    np.testing.assert_allclose((left_position + right_position) / 2.0, 0.0, atol=1e-12)
    assert right_position[0] > left_position[0]
    # Physical up (-Z in left base) becomes +Z in midpoint world.
    np.testing.assert_allclose(
        world["T_world_left_base"][:3, :3] @ np.array((0.0, 0.0, -1.0)),
        (0.0, 0.0, 1.0),
        atol=1e-12,
    )


def test_serialized_pose_includes_fixed_axis_rpy_for_mount_config():
    pose = transform((0.5, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    serialized = transform_to_pose(pose)
    np.testing.assert_allclose(
        serialized["rotation_rpy_rad"], (math.pi, 0.0, 0.0), atol=1e-12
    )


def test_interactive_configuration_selects_mode_pads_and_sample_count():
    config = {"sampling": {"minimum_samples": 5, "recommended_samples": 8}}
    with patch("builtins.input", side_effect=("1", "right", "2", "1", "6")):
        selection = _configuration(config)
    assert selection == {
        "mode": "1",
        "reference_robot": "right",
        "left_pad_frame": "left/ag95/right_pad_surface",
        "right_pad_frame": "right/ag95/left_pad_surface",
        "sample_count": 6,
    }


def test_zenoh_paths_are_resolved_against_workspace(monkeypatch):
    monkeypatch.setenv("ZENOH_SESSION_CONFIG_URI", "./zenoh/client.json5")
    monkeypatch.setenv("ZENOH_ROUTER_CONFIG_URI", "./zenoh/router.json5")
    _normalize_zenoh_config_paths()
    assert Path(os.environ["ZENOH_SESSION_CONFIG_URI"]).is_file()
    assert Path(os.environ["ZENOH_ROUTER_CONFIG_URI"]).is_file()
