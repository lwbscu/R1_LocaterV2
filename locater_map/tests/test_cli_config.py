from argparse import Namespace

from main import _dt35_correct_lidar_frames


def test_cli_fusion_uses_display_dt35_lidar_correction_default():
    args = Namespace(fusion_dt35_correct_lidar_frames=False)
    config = {"display": {"live_fusion_dt35_correct_lidar_frames": True}}

    assert _dt35_correct_lidar_frames(args, config) is True


def test_cli_fusion_flag_can_force_dt35_lidar_correction_on():
    args = Namespace(fusion_dt35_correct_lidar_frames=True)
    config = {"display": {"live_fusion_dt35_correct_lidar_frames": False}}

    assert _dt35_correct_lidar_frames(args, config) is True
