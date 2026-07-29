"""Line trajectory generator for linear axis sweeps in the air with the end effector."""

import time

import numpy as np

from simulation.cartesian_ik import CartesianIK


class LineTrajectoryGenerator:
    """Generates and executes linear Cartesian trajectories along X, Y, and Z axes.

    Supports both MuJoCo simulation (`SO101Robot`) and real hardware (`RealSO101Robot`).
    """

    def __init__(self, robot, ik_solver: CartesianIK | None = None):
        """Initialize trajectory generator.

        Args:
            robot: SO101Robot or RealSO101Robot instance.
            ik_solver: Optional CartesianIK solver.
        """
        self.robot = robot
        self.ik_solver = ik_solver if ik_solver is not None else CartesianIK()

    def generate_line_waypoints(
        self,
        start_pos: np.ndarray,
        axis: str = "y",
        distance: float = 0.20,
        num_points: int = 50,
    ) -> list[np.ndarray]:
        """Generate 3D waypoints forming a straight line sweep from -half to +half.

        Args:
            start_pos: Starting central 3D position (x, y, z) in meters.
            axis: Axis direction ('x', 'y', or 'z').
            distance: Total length of line sweep in meters (e.g. 0.20 m = 20 cm).
            num_points: Number of discrete linear interpolation steps.

        Returns:
            List of 3D target points forming a complete forward and back line trajectory.
        """
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_idx = axis_map.get(axis.lower(), 1)

        half_dist = distance / 2.0
        waypoints = []

        # Forward segment: sweep from (center - 10cm) to (center + 10cm)
        for t in np.linspace(-half_dist, half_dist, num_points):
            pt = start_pos.copy()
            pt[axis_idx] += t
            waypoints.append(pt)

        # Backward segment: sweep back from (center + 10cm) to (center - 10cm)
        for t in np.linspace(half_dist, -half_dist, num_points):
            pt = start_pos.copy()
            pt[axis_idx] += t
            waypoints.append(pt)

        return waypoints

    def execute_line_sweep(
        self,
        axis: str = "y",
        distance: float = 0.20,
        num_points: int = 50,
        step_delay: float = 0.03,
    ) -> None:
        """Execute a linear Cartesian axis sweep in the air using Inverse Kinematics.

        Args:
            axis: Axis to move along ('x', 'y', or 'z').
            distance: Sweep distance along axis in meters.
            num_points: Number of waypoints along the line trajectory.
            step_delay: Delay between waypoint steps in seconds.
        """
        # Determine starting pose end-effector position
        if hasattr(self.robot, "get_end_effector_pose"):
            start_ee_pos, _ = self.robot.get_end_effector_pose()
        else:
            # Fallback if real robot does not report end-effector pose directly
            qpos_curr = self.robot.get_joint_positions()
            self.ik_solver.solve_ik(np.array([0.18, 0.0, 0.15]), initial_qpos=qpos_curr)
            start_ee_pos = self.ik_solver.data.site_xpos[self.ik_solver.site_id].copy()

        waypoints = self.generate_line_waypoints(
            start_pos=start_ee_pos,
            axis=axis,
            distance=distance,
            num_points=num_points,
        )

        dist_cm = distance * 100
        print(f"Executing line sweep along {axis.upper()}-axis ({dist_cm:.1f} cm)...")
        current_qpos = self.robot.get_joint_positions()

        for idx, target_3d in enumerate(waypoints):
            # Check if viewer was closed by user
            if (
                hasattr(self.robot, "viewer")
                and self.robot.viewer is not None
                and not self.robot.viewer.is_running()
            ):
                return

            # Compute inverse kinematics for current target point
            target_qpos = self.ik_solver.solve_ik(
                target_pos=target_3d,
                initial_qpos=current_qpos,
                max_iters=50,
                tol=1e-3,
            )

            # Command robot
            self.robot.set_joint_positions(target_qpos)

            # Advance simulation physics if available
            if hasattr(self.robot, "step"):
                self.robot.step(num_steps=10)

            # Sync viewer window if viewer reference was provided
            if hasattr(self.robot, "viewer") and self.robot.viewer is not None:
                self.robot.viewer.sync()

            current_qpos = target_qpos
            time.sleep(step_delay)

        print(f"Line sweep along {axis.upper()}-axis completed successfully.")
