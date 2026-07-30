# LeDroid-101 Simulation & Hardware Guide

This directory documents the **MuJoCo physics simulation** environment, **Digital Twin control**, and **real hardware Cartesian control** for the SO-101 robot arm.

---

## 1. Quickstart

To run the unified CLI, use `uv run python main.py` with your chosen execution mode (`sim`, `real`, or `twin`).

### **Execution Modes (`--mode`)**
- `--mode sim`: Runs purely in MuJoCo 3D simulation.
- `--mode real`: Controls the physical SO-101 robot arm directly via serial port.
- `--mode twin`: **Digital Twin Mode!** Launches the MuJoCo 3D viewer and streams simulation joint states live to the physical SO-101 robot arm in real time.

---

### **Task Routines (`--task`)**

#### **1. Cartesian Line Sweep (`--task line`)**
Move the end-effector along smooth, vibration-free straight lines in Cartesian 3D space across X, Y, and Z axes to test inverse kinematics, posture stability, and joint limits.

Featuring real-time RGB coordinate frame visualization (`mjFRAME_SITE`), seamless out-and-back trajectory loops, and under-actuated 5-DOF damped least-squares IK with null-space posture anchoring towards an ergonomic mid-air `SWEEP_HOME` configuration:

![Cartesian Line Sweep Simulation Video Demo](sweep_fixed.mp4)

```bash
# Pure simulation with live interactive 3D GUI viewer
uv run python main.py --mode sim --task line

# Headless video recording to disk (no GUI needed!)
uv run python main.py --mode sim --task line --record --record-path docs/sweep_fixed.mp4

# Digital Twin Mode (moves physical arm live matching MuJoCo viewer!)
uv run python main.py --mode twin --port /dev/tty.usbmodem1201 --task line
```

#### **2. Preset Pose Trajectory Cycle (`--task poses`)**
Interpolate smoothly across `HOME` -> `REACH` -> `PICK` -> `STOW` -> `HOME` poses:
```bash
# Digital Twin Mode
uv run python main.py --mode twin --port /dev/tty.usbmodem1201 --task poses
```

#### **3. Manual Pose Inspection Mode (`--task manual`)**
Drag robot joint actuators using the MuJoCo viewer control sliders and mirror physical motor positions live:
```bash
# Digital Twin Manual Mode (Control physical arm directly via MuJoCo sliders!)
uv run python main.py --mode twin --port /dev/tty.usbmodem1201 --task manual
```
- Use the **Control** / **Actuators** sliders on the right-hand panel in the viewer window to pose the arm.
- Terminal automatically outputs `Joint Positions (rad)`, `Actuator Control (rad)`, and `End-Effector XYZ (m)` every 2.5 seconds.

#### **4. Interactive 3D GUI Simulation Viewer**
Launch passive 3D MuJoCo visualizer with keyboard controls:
```bash
uv run python main.py --mode sim --task interactive
```

**Interactive Key Bindings:**
- `1`: Move to `HOME` pose
- `2`: Move to `REACH` pose
- `3`: Move to `PICK` pose
- `4`: Move to `STOW` pose
- `X`: Execute linear end-effector sweep along X-axis
- `Y`: Execute linear end-effector sweep along Y-axis
- `Z`: Execute linear end-effector sweep along Z-axis
- `Space`: Toggle Gripper Open / Closed

---

### **Real Physical Robot Mode (`--mode real`)**

> **Safety Notice:** Ensure the SO-101 robot arm is placed on a clear table surface with adequate clearance before triggering motion routines.

#### **1. Execute Cartesian Line Sweep on Real Arm**
Connect your SO-101 robot via USB serial adapter and specify your serial port:
```bash
uv run python main.py --mode real --port /dev/tty.usbmodem1201 --task line --axis y
```

#### **2. Execute Preset Pose Cycle on Real Arm**
```bash
uv run python main.py --mode real --port /dev/tty.usbmodem1201 --task poses
```

---

## 2. Diagnostics & Testing Utilities

You can run isolated diagnostic checks and benchmarks:

```bash
# System diagnostic check (Python, MuJoCo, IK, LeRobot driver)
uv run python -m simulation.check_env

# Verify joint pose interpolation tracking
uv run python -m simulation.test_poses

# Test GPU/CPU parallel batched simulation (MuJoCo Warp)
uv run python -m simulation.test_warp
```

---

## 3. Architecture & Package Modules

```text
simulation/
├── assets/                  # MJCF definitions, scene XML, and 3D STL meshes
│   ├── scene.xml            # World environment scene file
│   ├── so101.xml            # SO-101 6-DOF robot arm MJCF model
│   ├── joints_properties.xml# STS3215 position actuator specs
│   └── meshes/              # 13 STL CAD model meshes
│
├── robot.py                 # SO101Robot controller, preset poses (SWEEP_HOME), & video recorder
├── real_robot.py            # RealSO101Robot hardware interface using lerobot MotorsBus
├── twin_robot.py            # TwinSO101Robot digital twin controller mirroring sim to real
├── cartesian_ik.py          # 5-DOF decoupled DLS Inverse Kinematics with posture projection
├── line_trajectory.py       # Continuous closed-loop piecewise-linear waypoint generator
├── env.py                   # SO101ReachEnv standard Gymnasium RL environment
├── warp_env.py              # SO101WarpEnv parallel GPU/CPU batched simulator
├── simulate.py              # Interactive 3D passive viewer launcher
├── check_env.py             # Diagnostic and benchmarking script
├── test_poses.py            # Joint trajectory tracking unit test
└── test_warp.py             # MuJoCo Warp throughput test
```
