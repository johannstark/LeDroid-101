"""Interactive 3D simulation viewer launcher for SO-101 in MuJoCo."""

import os
import sys

import mujoco
import mujoco.viewer

from simulation.line_trajectory import LineTrajectoryGenerator
from simulation.robot import PRESET_POSES, SO101Robot


def ensure_mjpython() -> None:
    """Relaunch under mjpython on macOS if running under default CPython."""
    if sys.platform != "darwin":
        return

    if os.environ.get("MJPYTHON_RUNNING") == "1":
        return

    import shutil

    mjpython_path = shutil.which("mjpython")
    if not mjpython_path:
        python_dir = os.path.dirname(sys.executable)
        candidate = os.path.join(python_dir, "mjpython")
        if os.path.isfile(candidate):
            mjpython_path = candidate

    if mjpython_path:
        os.environ["MJPYTHON_RUNNING"] = "1"
        os.execv(mjpython_path, [mjpython_path] + sys.argv)


def main(
    robot: SO101Robot | None = None,
    record: bool = False,
    record_path: str = "recordings/video.mp4",
) -> None:
    """Launch interactive 3D simulation viewer with keyboard control callbacks.

    Args:
        robot: Optional robot instance (SO101Robot or TwinSO101Robot).
        record: If True, record execution frames to a video file.
        record_path: Destination path for output video file.
    """
    ensure_mjpython()

    print("Launching SO-101 MuJoCo Interactive Simulation Viewer...")
    print("-------------------------------------------------------")
    print("Keyboard Controls:")
    print("  [1] Move to HOME pose")
    print("  [2] Move to REACH pose")
    print("  [3] Move to PICK pose")
    print("  [4] Move to STOW pose")
    print("  [X] Execute End-Effector Line Sweep along X-axis")
    print("  [Y] Execute End-Effector Line Sweep along Y-axis")
    print("  [Z] Execute End-Effector Line Sweep along Z-axis")
    print("  [Space] Toggle Gripper Open / Closed")
    print("-------------------------------------------------------")

    if robot is None:
        robot = SO101Robot()
    trajectory_gen = LineTrajectoryGenerator(robot)

    # State flags
    gripper_open = True
    active_action = None

    def key_callback(keycode: int) -> None:
        nonlocal gripper_open, active_action

        # Convert ASCII keycode
        try:
            char = chr(keycode).upper()
        except ValueError:
            return

        if char == "1":
            print("Moving to HOME pose...")
            robot.move_to_pose(PRESET_POSES["HOME"], duration_sec=0.8)
        elif char == "2":
            print("Moving to REACH pose...")
            robot.move_to_pose(PRESET_POSES["REACH"], duration_sec=0.8)
        elif char == "3":
            print("Moving to PICK pose...")
            robot.move_to_pose(PRESET_POSES["PICK"], duration_sec=0.8)
        elif char == "4":
            print("Moving to STOW pose...")
            robot.move_to_pose(PRESET_POSES["STOW"], duration_sec=0.8)
        elif char in ["X", "Y", "Z"]:
            active_action = char.lower()
        elif char == " ":
            gripper_open = not gripper_open
            qpos = robot.get_joint_positions()
            qpos[5] = 0.4 if gripper_open else -0.2
            robot.set_joint_positions(qpos)
            print(f"Gripper {'Opened' if gripper_open else 'Closed'}.")

    robot.reset("HOME")

    if record and hasattr(robot, "start_recording"):
        print(f"Starting video recording to {record_path}...")
        robot.start_recording(record_path)

    try:
        with mujoco.viewer.launch_passive(
            robot.model, robot.data, key_callback=key_callback
        ) as viewer:
            robot.viewer = viewer
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
            while viewer.is_running():
                if active_action in ["x", "y", "z"]:
                    axis = active_action
                    active_action = None
                    trajectory_gen.execute_line_sweep(axis=axis, distance=0.08, num_points=30)

                robot.step(1)
                viewer.sync()
    finally:
        if record and hasattr(robot, "stop_recording"):
            robot.stop_recording()


if __name__ == "__main__":
    main()
