"""Test preset poses and smooth joint trajectory interpolation."""

import numpy as np

from simulation.robot import PRESET_POSES, SO101Robot


def test_poses() -> None:
    """Test preset poses and joint position tracking accuracy."""
    print("Testing SO-101 Preset Poses and Interpolation Trajectories...")
    robot = SO101Robot()
    robot.reset("HOME")

    poses_sequence = ["HOME", "REACH", "PICK", "STOW", "HOME"]

    for pose_name in poses_sequence:
        print(f"Moving to {pose_name} pose...")
        target_qpos = PRESET_POSES[pose_name]
        robot.move_to_pose(target_qpos, duration_sec=0.5)

        curr_qpos = robot.get_joint_positions()
        error = np.max(np.abs(curr_qpos - target_qpos))
        print(f"  Current qpos: {np.round(curr_qpos, 3)}")
        print(f"  Target qpos:  {np.round(target_qpos, 3)}")
        print(f"  Max Joint Tracking Error: {error:.4f} rad")
        assert error < 0.25, f"Tracking error too high for pose {pose_name}: {error}"

    print("Preset Poses Verification Test PASSED!")


if __name__ == "__main__":
    test_poses()
