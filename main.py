"""Unified CLI entry point for LeDroid-101 (Simulation & Physical Hardware)."""

import argparse
import sys
import time

from simulation.cartesian_ik import CartesianIK
from simulation.line_trajectory import LineTrajectoryGenerator
from simulation.real_robot import RealSO101Robot
from simulation.robot import PRESET_POSES, SO101Robot


def main() -> None:
    """Execute main CLI for LeDroid-101 (Simulation & Physical Hardware)."""
    parser = argparse.ArgumentParser(
        description="LeDroid-101 Main CLI — Run MuJoCo simulation or physical SO-101 commands."
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["sim", "real", "twin"],
        default="sim",
        help=(
            "Execution target mode: 'sim' (MuJoCo simulation), 'real' (Physical SO-101 arm), "
            "or 'twin' (Sim + Real Digital Twin Mirror)."
        ),
    )
    parser.add_argument(
        "--task",
        type=str,
        choices=["line", "poses", "interactive", "manual"],
        default="line",
        help=(
            "Task routine to execute: 'line' (Cartesian linear sweeps), "
            "'poses' (Preset pose cycle), 'interactive' (3D visualizer), or "
            "'manual' (Manual viewer sliders for Joint/Actuator inspection)."
        ),
    )
    parser.add_argument(
        "--axis",
        type=str,
        choices=["x", "y", "z"],
        default="y",
        help="Axis to sweep along for 'line' task ('x', 'y', or 'z').",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=0.20,
        help="Line sweep total distance in meters (default: 0.20 m, -10 cm to +10 cm).",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/tty.usbmodem1201",
        help="Serial port for real physical arm (e.g., /dev/tty.usbmodem1201 or /dev/ttyUSB0).",
    )

    args = parser.parse_args()

    print("=" * 60)
    print(f"LeDroid-101 Execution — Mode: [{args.mode.upper()}] | Task: [{args.task.upper()}]")
    print("=" * 60)

    # 1. Launch Interactive GUI viewer if requested in sim mode
    if args.task in ["interactive", "manual"]:
        if args.mode == "real":
            print("Error: Visualizer tasks are only available in 'sim' or 'twin' mode.")
            sys.exit(1)

        if args.task == "manual":
            from simulation.simulate import ensure_mjpython

            ensure_mjpython()
            import mujoco
            import mujoco.viewer

            print("=" * 60)
            print("Launching SO-101 Manual Pose Inspection Viewer")
            print("------------------------------------------------------------")
            print("Use the 'Control' / 'Actuators' sliders on the right panel in MuJoCo")
            print("to manually move the robot joints to desired poses.")
            print("Press [P] or [Space] in terminal / viewer to print current joint values!")
            print("Close the viewer window when done.")
            print("=" * 60)

            if args.mode == "twin":
                from simulation.twin_robot import TwinSO101Robot

                robot = TwinSO101Robot(port=args.port)
            else:
                robot = SO101Robot()

            robot.reset("HOME")

            last_print_time = 0.0

            with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
                viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
                robot.viewer = viewer
                while viewer.is_running():
                    # Step physics so actuator controls drive qpos and mirror to real robot
                    robot.step(1)
                    viewer.sync()
                    time.sleep(0.01)

                    curr_time = time.time()
                    if curr_time - last_print_time > 2.5:
                        qpos = robot.get_joint_positions()
                        ctrl = robot.data.ctrl.copy()
                        ee_pos, _ = robot.get_end_effector_pose()

                        qpos_str = ", ".join([f"{v:.3f}" for v in qpos])
                        ctrl_str = ", ".join([f"{v:.3f}" for v in ctrl])
                        ee_str = ", ".join([f"{v:.3f}" for v in ee_pos])

                        print("\n[CURRENT POSE STATE]")
                        print(f" Joint Positions (rad) [qpos]: [{qpos_str}]")
                        print(f" Actuator Control (rad) [ctrl]: [{ctrl_str}]")
                        print(f" End-Effector XYZ (m)   [ee_pos]: [{ee_str}]")
                        last_print_time = curr_time

            print("\nSetting robot to HOME pose before exit...")
            if hasattr(robot, "move_to_pose"):
                robot.move_to_pose(PRESET_POSES["HOME"], duration_sec=1.0)
            else:
                robot.set_joint_positions(PRESET_POSES["HOME"])

            if hasattr(robot, "disconnect"):
                robot.disconnect()

            print("\nManual inspection session finished.")
            return

        from simulation.simulate import main as launch_simulate

        launch_simulate()
        return

    # 2. Instantiate target robot instance (Sim, Real, or Twin)
    if args.mode in ["sim", "twin"]:
        from simulation.simulate import ensure_mjpython

        ensure_mjpython()
        import mujoco.viewer

        if args.mode == "twin":
            from simulation.twin_robot import TwinSO101Robot

            robot = TwinSO101Robot(port=args.port)
        else:
            robot = SO101Robot()

        robot.reset("HOME")
        viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        robot.viewer = viewer
        print("MuJoCo Passive Viewer launched.")
    else:
        print(f"Connecting to real SO-101 robot arm on port {args.port}...")
        robot = RealSO101Robot(port=args.port)

    ik_solver = CartesianIK()
    trajectory_gen = LineTrajectoryGenerator(robot=robot, ik_solver=ik_solver)

    # Helper to check if execution loop should continue
    def is_running() -> bool:
        if args.mode in ["sim", "twin"] and hasattr(robot, "viewer") and robot.viewer is not None:
            return robot.viewer.is_running()
        return True

    # 3. Execute requested task in continuous loop until viewer closes / interrupted
    print("Starting continuous execution loop (close viewer window or press Ctrl+C to stop)...")
    try:
        if args.task == "poses":
            pose_list = ["HOME", "REACH", "PICK", "STOW", "HOME"]
            cycle_count = 0
            while is_running():
                cycle_count += 1
                print(f"\n--- Starting Pose Cycle #{cycle_count} ---")
                for pose_name in pose_list:
                    if not is_running():
                        break
                    print(f" -> Transitioning to {pose_name} pose...")
                    target_qpos = PRESET_POSES[pose_name]
                    if hasattr(robot, "move_to_pose"):
                        robot.move_to_pose(target_qpos, duration_sec=1.0)
                    else:
                        robot.set_joint_positions(target_qpos)
                        time.sleep(1.0)

        elif args.task == "line":
            axes_sequence = ["x", "y", "z"]
            axis_cycle = 0

            # First transition from HOME to MIDDLE pose before starting axis sweeps
            if is_running():
                print("\nTransitioning from HOME pose to MIDDLE pose before starting sweeps...")
                if hasattr(robot, "move_to_pose"):
                    robot.move_to_pose(PRESET_POSES["MIDDLE"], duration_sec=1.2)
                else:
                    robot.set_joint_positions(PRESET_POSES["MIDDLE"])
                    time.sleep(1.2)

            while is_running():
                axis_cycle += 1
                current_axis = axes_sequence[(axis_cycle - 1) % len(axes_sequence)]
                seq_msg = f"\n--- Axis Sequence [{current_axis.upper()}] (Cycle #{axis_cycle}) ---"
                print(seq_msg)

                # Perform 3 sweeps along current axis
                for sweep_i in range(1, 4):
                    if not is_running():
                        break
                    print(f"Sweep {sweep_i}/3 along {current_axis.upper()}-axis...")
                    trajectory_gen.execute_line_sweep(
                        axis=current_axis,
                        distance=args.distance,
                        num_points=40,
                        step_delay=0.03,
                    )

                # Move back to MIDDLE pose before changing axis
                if is_running():
                    print("Returning to MIDDLE pose before switching axis...")
                    if hasattr(robot, "move_to_pose"):
                        robot.move_to_pose(PRESET_POSES["MIDDLE"], duration_sec=0.8)
                    else:
                        robot.set_joint_positions(PRESET_POSES["MIDDLE"])
                        time.sleep(0.8)

    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")

    # Always park robot safely to HOME pose on completion/exit
    print("\nSafely parking robot to HOME pose...")
    if hasattr(robot, "move_to_pose"):
        robot.move_to_pose(PRESET_POSES["HOME"], duration_sec=1.0)
    else:
        robot.set_joint_positions(PRESET_POSES["HOME"])

    # Clean up simulation viewer or real hardware connection
    if args.mode in ["sim", "twin"] and hasattr(robot, "viewer") and robot.viewer is not None:
        if robot.viewer.is_running():
            robot.viewer.close()

    if hasattr(robot, "disconnect"):
        robot.disconnect()

    print("Task execution finished successfully.")


if __name__ == "__main__":
    main()
