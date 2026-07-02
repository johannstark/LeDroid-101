# Assembly & Calibration of the SO-101 Robot Arm

This guide details how to configure Feetech STS3215 servo motors, physically assemble the SO-101 robot arm, and calibrate the joints so both **Follower** and **Leader** arms synchronize position values perfectly.

---

## 1. Setting Servo Motor IDs

Before assembling the physical joints, each Feetech STS3215 servo must be programmed with a unique identifier (ID) and a matching communication baudrate.

### Motor Gearing Summary

| Joint | Name | Motor ID | Gearing (Follower) | Gearing (Leader) |
| --- | --- | --- | --- | --- |
| Joint 1 | Base/Shoulder Pan | 1 | 1/345 | 1/191 |
| Joint 2 | Shoulder Lift | 2 | 1/345 | 1/345 |
| Joint 3 | Elbow Flex | 3 | 1/345 | 1/191 |
| Joint 4 | Wrist Flex | 4 | 1/345 | 1/147 |
| Joint 5 | Wrist Roll | 5 | 1/345 | 1/147 |
| Joint 6 | Gripper | 6 | 1/345 | 1/147 |

### ID Setup Procedure

All STS3215 motors ship with the default **ID 1**. You must configure each motor **one by one** to write their assigned IDs to the motor's EEPROM (non-volatile memory).

> [!Important]
> Do not chain them together during this step.

1. **Find USB Ports**:
   Connect your MotorBus adapter to the computer via USB and external power. Stop all utilities and run:

   ```bash
   lerobot-find-port
   ```

   Follow the on-screen instructions (e.g., unplug and replug when prompted) to locate the active USB link.

   Example discovered port path: `/dev/tty.usbmodem575E0032081`.

2. **Run ID Configuration Interactively**:
   To set motor IDs for the **Follower** arm, run:

   ```bash
   lerobot-setup-motors --robot.type=so101_follower --robot.port=<your_follower_port>
   ```

   For the **Leader** arm, run:

   ```bash
   lerobot-setup-motors --teleop.type=so101_leader --teleop.port=<your_leader_port>
   ```

3. **Follow the On-Screen Sequence**:

   - The CLI tool will prompt you to configure each joint in the following sequence:
     **Gripper (ID 6) $\rightarrow$ Wrist Roll (ID 5) $\rightarrow$ Wrist Flex (ID 4) $\rightarrow$ Elbow Flex (ID 3) $\rightarrow$ Shoulder Lift (ID 2) $\rightarrow$ Shoulder Pan (ID 1)**.
   - For each joint/motor:
     1. Unplug current motor (if any).
     2. Connect a single, unconfigured motor with default ID 1.
     3. Press **Enter** in the terminal to program the motor to the active ID.
     4. Mark or label the motor (e.g., "ID 6") so you don't mix them up.
   - Once all 6 motors are individually configured, you can daisy-chain them together and connect the first motor to the controller board.

---

## 2. Assembling the SO-101 Robot Arm

Ensure all 3D-printed parts have support structures cleared using a flat-head screwdriver or modeling clippers.

### Required Hardware

- 6× configured Feetech STS3215 motors
- M2×6mm screws
- M3×6mm screws
- Metal/Plastic motor horns

> [!Note]
> All screws are included with the motors. You won't need anything else.

### Joint-by-Joint Assembly Step-by-Step

#### Joint 1 (Base / Shoulder Pan)

- Install two motor horns on motor ID 1; secure the top side with an M3×6mm screw.
- Place motor ID 1 into the 3D-printed main base plate.
- Secure with 4 M2×6mm screws (2 on top side, 2 on the bottom side).
- Slide the motor holder bracket over the motor housing and fasten with 2 M2×6mm screws.
- Attach the Shoulder part with 4 M3×6mm screws on both top and bottom.
- Bolt on the Shoulder motor holder.

#### Joint 2 (Shoulder Lift)

- Install two motor horns on motor ID 2; secure top with an M3×6mm screw.
- Slide motor ID 2 into the Shoulder holder from the top.
- Secure the motor with 4 M2×6mm screws.
- Connect the Upper Arm part using 4 M3×6mm screws per side.

#### Joint 3 (Elbow Flex)

- Install two motor horns on motor ID 3; secure top with an M3×6mm screw.
- Insert and secure the motor in the Arm frame with 4 M2×6mm screws.
- Connect the Forearm part using 4 M3×6mm screws per side.

#### Joint 4 (Wrist Flex)

- Install two motor horns on motor ID 4; secure top with an M3×6mm screw.
- Slide the Wrist motor holder over the motor housing.
- Fasten with 4 M2×6mm screws.

#### Joint 5 (Wrist Roll)

- Insert motor ID 5 into the Wrist holder structure; secure it using 2 M2×6mm screws.
- Install one motor horn on motor ID 5; secure with an M3×6mm screw.
- Bolt the entire Wrist unit to the Joint 4 output horn using 4 M3×6mm screws per side.

#### Joint 6 (Gripper / Handle)

- Secure the main Gripper frame to the motor ID 5 output horn using 4 M3×6mm screws.
- Insert the Gripper motor (ID 6); secure it using 2 M2×6mm screws per side.
- Install motor horns on biological sides of motor ID 6; secure with an M3×6mm screw on top.
- Connect the Gripper claw linkage gears using 4 M3×6mm screws per side.

---

## 3. Calibration Procedures

The calibration script measures the physical range of motion of each joint to ensure that both the **Follower** and **Leader** arms speak the exact same coordinate/angle values.

### Execution Commands

To calibrate the **Follower** arm, run:

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=<your_follower_port_path> \
  --robot.id=<your_custom_follower_id>
```

To calibrate the **Leader** hand/arm, run:

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=<your_leader_port_path> \
  --teleop.id=<your_custom_leader_id>
```

### Steps for Physical Calibration

1. **Central Position Alignment**:
   When starting the script, you will be prompted to position the robot arm at the center of its range of motion. Align every motor to its mechanical middle. Press **Enter** once complete.

2. **Range of Motion Evaluation**:
   The script will instruct you to manually move each joint through its entire range of motion (from lowest possible angle to highest possible angle). Move joints slowly to prevent damage to gear structures.

3. **Validation & Configuration File Saving**:
   The utility tracks the minimum and maximum encoders automatically. It will terminate and save the calibration configurations to your local file path under the specified `--robot.id` or `--teleop.id` parameters.

---

See also:

- [docs/installation.md](docs/installation.md) — For environment and package management instruction.
- [README.md](README.md) — Landing page of the project.
