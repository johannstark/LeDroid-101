# LeDroid-101

My approach to HuggingFace's LeRobot and SO-101 Robot Arm.

---

## Welcome to LeDroid-101 🤖

LeDroid-101 is my custom approach for replicating, learning, and expanding HuggingFace's SO-101 Robot arm setup. This repository is intended to be replicated by anyone to learn everything about configuring, assembling, calibrating, and using this robot in a simplified and friendly way.

We use Python version **3.13.7** and the extremely fast and deterministic package manager `uv` for managing all environments and dependencies.

---

## Demonstration: Precision Cartesian Line Sweeps & 5-DOF IK

Watch the SO-101 robotic arm execute precision linear trajectories across the X, Y, and Z axes in MuJoCo simulation! Featuring real-time RGB end-effector coordinate frame visualization, under-actuated 5-DOF position tracking, and null-space joint posture stabilization from an ergonomic mid-air `SWEEP_HOME` pose:

![SO-101 Cartesian Line Sweep Demo in MuJoCo](docs/sweep_fixed.mp4)

> **Try it yourself!** Run the headless simulation recording directly via:
> ```bash
> uv run python main.py --mode sim --task line --record --record-path docs/sweep_fixed.mp4
> ```

---

## Getting Started

To get started on your own replica of the SO-101 Robot Arm, we recommend reading the reference files in order. Choose the document based on your active setup stage:

### Reference Guides & Documentation

| Step | Topic | Reference File | Focus |
| --- | --- | --- | --- |
| **1** | **Basics & Installation** | [docs/installation.md](docs/installation.md) | Installing Python 3.13.7, configuring `uv`, and syncing the environment defined in [pyproject.toml](pyproject.toml). |
| **2** | **Motors & Gearing** | [docs/assembly.md](docs/assembly.md#1-setting-servo-motor-ids) | Individual servo motor ID mapping (IDs 1-6) and baud rate options via `lerobot-find-port`. |
| **3** | **Physical Assembly** | [docs/assembly.md](docs/assembly.md#2-assembling-the-so-101-robot-arm) | Step-by-step assembly of STS3215 joints from Joint 1 (Base) to Joint 6 (Gripper). |
| **4** | **Calibration** | [docs/assembly.md](docs/assembly.md#3-calibration-procedures) | How to run the `lerobot-calibrate` script for both Leader and Follower configurations. |
| **5** | **LeKinematics & IK Math** | [lekinematics/theory.md](lekinematics/theory.md) <br> [docs/kinematics_math.md](docs/kinematics_math.md) <br> [lekinematics/le_kinematics.py](lekinematics/le_kinematics.py) | Forward/inverse kinematics theory, spatial Jacobians, DLS optimization, 5-DOF decoupling, and practical Python examples. |
| **6** | **MuJoCo Sim, Digital Twin & Real Control** | [docs/simulation.md](docs/simulation.md) <br> [main.py](main.py) | MuJoCo physics simulation, Gymnasium RL environment, 3D interactive viewer, Digital Twin mode (`--mode twin`), video recording (`--record`), and Cartesian end-effector line routines. |

---

## Contributing and Community

This project is fully open-source and welcoming replication. Feel free to open issues or Pull Requests with your own examples and use cases!

---

Made in Colombia 🇨🇴 with ❤️
