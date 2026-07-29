"""Cartesian Inverse Kinematics (IK) solver for the SO-101 robot arm."""

from pathlib import Path

import mujoco
import numpy as np


class CartesianIK:
    """Numerical Inverse Kinematics solver using MuJoCo Jacobian damped least-squares.

    Attributes:
        model: MuJoCo MjModel instance.
        data: MuJoCo MjData instance.
        site_id: End effector site ID.
        joint_ids: List of joint IDs.
    """

    def __init__(self, xml_path: str | Path | None = None):
        """Initialize Cartesian IK solver for SO-101 model.

        Args:
            xml_path: Path to the MJCF scene XML file. Defaults to simulation assets/scene.xml.
        """
        if xml_path is None:
            xml_path = Path(__file__).parent / "assets" / "scene.xml"

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]
        self.joint_ids = [self.model.joint(name).id for name in joint_names]

        try:
            self.site_id = self.model.site("gripperframe").id
        except KeyError:
            self.site_id = self.model.site("end_effector").id

    def solve_ik(
        self,
        target_pos: np.ndarray,
        initial_qpos: np.ndarray | None = None,
        max_iters: int = 200,
        tol: float = 1e-4,
        damping: float = 1e-3,
    ) -> np.ndarray:
        """Solve numerical inverse kinematics for target 3D end-effector position.

        Args:
            target_pos: Target (x, y, z) 3D coordinate array in meters.
            initial_qpos: Starting joint angles configuration array.
            max_iters: Maximum IK optimization iterations.
            tol: Target distance error threshold in meters.
            damping: Damped least-squares regularization factor.

        Returns:
            A numpy array of 6 joint angles in radians.
        """
        if initial_qpos is not None:
            for i, j_id in enumerate(self.joint_ids):
                qpos_adr = self.model.jnt_qposadr[j_id]
                self.data.qpos[qpos_adr] = initial_qpos[i]

        mujoco.mj_forward(self.model, self.data)

        # Allocate Jacobian buffers (3 x nv for site position)
        jacp = np.zeros((3, self.model.nv))

        for _ in range(max_iters):
            current_pos = self.data.site_xpos[self.site_id].copy()
            err = target_pos - current_pos

            if np.linalg.norm(err) < tol:
                break

            # Compute site position Jacobian
            mujoco.mj_jacSite(self.model, self.data, jacp, None, self.site_id)

            # Filter Jacobian columns corresponding only to our 6 active joints
            dof_indices = [self.model.jnt_dofadr[j_id] for j_id in self.joint_ids]
            J = jacp[:, dof_indices]

            # Damped least squares: dq = J_T * inv(J * J_T + lambda^2 * I) * err
            JJt = J @ J.T + (damping**2) * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, err)

            # Update qpos
            for i, j_id in enumerate(self.joint_ids):
                qpos_adr = self.model.jnt_qposadr[j_id]
                self.data.qpos[qpos_adr] += dq[i]

                # Enforce joint position limits
                jnt_range = self.model.jnt_range[j_id]
                self.data.qpos[qpos_adr] = np.clip(
                    self.data.qpos[qpos_adr], jnt_range[0], jnt_range[1]
                )

            mujoco.mj_forward(self.model, self.data)

        qpos_res = []
        for j_id in self.joint_ids:
            qpos_adr = self.model.jnt_qposadr[j_id]
            qpos_res.append(self.data.qpos[qpos_adr])

        return np.array(qpos_res, dtype=np.float32)
