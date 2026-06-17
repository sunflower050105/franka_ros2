# franka_velocity_ctrl

A ROS 2 package for **joint-velocity control** of the **Franka Research 3 (FR3)** robotic arm on real hardware.

It provides two nodes:

| Node | Purpose |
|---|---|
| `velocity_command_node` | Reads a velocity profile (demo sine wave, external topic, or zero) and publishes commands to `joint_velocity_controller/commands` |
| `keyboard_teleop_node` | Lets you interactively drive individual joints from the terminal using keyboard keys |

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Hardware – Connecting the FR3](#2-hardware--connecting-the-fr3)
3. [Workspace Setup & Build](#3-workspace-setup--build)
4. [Package Structure](#4-package-structure)
5. [Nodes Reference](#5-nodes-reference)
   - [velocity_command_node](#velocity_command_node)
   - [keyboard_teleop_node](#keyboard_teleop_node)
6. [Launch the Package](#6-launch-the-package)
7. [Sending Velocity Commands Manually](#7-sending-velocity-commands-manually)
8. [Safety Notes](#8-safety-notes)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Prerequisites

### Software

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS (Jammy) |
| ROS 2 | Humble Hawksbill |
| `franka_ros2` | ≥ 0.1.15 (this workspace) |
| `ros2_control` / `velocity_controllers` | Humble |
| Python | 3.10+ |

Install ROS 2 controllers if not already present:

```bash
sudo apt update
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-velocity-controllers \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-controller-manager \
                 xterm       # required for keyboard teleop in launch
```

### FR3 Firmware

Make sure the Franka Research 3 is running **FCI-compatible firmware** (≥ 5.x).  
Check the [Franka World](https://franka.world/) or your robot's admin panel for the firmware version.

---

## 2. Hardware – Connecting the FR3

### Step 1 – Physical connection

1. Power on the FR3 base station and wait for the yellow LED to stop blinking.
2. Connect an **Ethernet cable** from the **robot's Control box shop port** to your PC's Ethernet port (or a dedicated network switch).

> **Tip:** Use a direct PC–robot connection for the lowest latency. If you must go through a switch, use a Gigabit switch only.

### Step 2 – Configure your PC's IP address

The FR3 ships with the default Control box IP **`172.16.0.2`**.  
Set your PC's Ethernet interface to the same subnet, e.g.:

```
IP address : 172.16.0.1
Netmask    : 255.255.255.0
Gateway    : (leave empty)
```

Using `nmcli` (recommended):

```bash
# Replace eth0 with your actual interface name (ip link show)
sudo nmcli connection add type ethernet ifname eth0 \
     ipv4.method manual ipv4.addresses 172.16.0.1/24 \
     connection.id franka-direct
sudo nmcli connection up franka-direct
```

### Step 3 – Verify connectivity

```bash
ping 172.16.0.2
```

You should see replies with < 1 ms RTT. If not, check cable, IP settings, or firewall rules.

### Step 4 – Unlock the robot via Desk

1. Open a browser and navigate to `https://172.16.0.2/desk`.
2. Log in (default: `admin` / `admin`).
3. Click **Activate FCI** (or release the brakes button, depending on firmware).
4. Make sure the status light on the robot turns **blue** (Ready / FCI active).

> ⚠️ The robot **must be in FCI mode** before launching any ROS 2 controller.

### Step 5 – Real-time kernel (recommended)

For stable 1 kHz control, install a PREEMPT_RT kernel:

```bash
sudo apt install linux-image-$(uname -r)-rt   # if available via apt
# or build from source – see Franka documentation
```

Set the real-time priorities in `/etc/security/limits.conf` (already provided in this repo):

```
@realtime    -    rtprio     99
@realtime    -    nice      -20
@realtime    -    memlock    unlimited
```

Add your user to the `realtime` group:

```bash
sudo usermod -aG realtime $USER
# log out and back in for the change to take effect
```

---

## 3. Workspace Setup & Build

```bash
# 1. Source ROS 2 base
source /opt/ros/humble/setup.bash

# 2. Install rosdep dependencies (run once from workspace root)
cd ~/franka_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the package (and its dependencies)
colcon build --packages-select franka_velocity_ctrl --symlink-install

# 4. Source the overlay
source install/setup.bash
```

> **Rebuild any time you edit Python files when not using `--symlink-install`.**

---

## 4. Package Structure

```
franka_velocity_ctrl/
├── config/
│   └── controllers.yaml          # ros2_control controller parameters
├── franka_velocity_ctrl/
│   ├── __init__.py
│   ├── velocity_command_node.py  # main velocity command publisher
│   └── keyboard_teleop_node.py   # keyboard-driven joint velocity teleop
├── launch/
│   └── fr3_velocity.launch.py    # main launch file
├── resource/
│   └── franka_velocity_ctrl      # ament resource index marker
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

---

## 5. Nodes Reference

### velocity_command_node

**Executable:** `velocity_command_node`  
**Default node name:** `velocity_command_node`

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/joint_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands sent to the hardware controller [rad/s] |
| `/velocity_command_node/target_velocities` | `std_msgs/Float64MultiArray` | Echo of the internally computed target (also subscription target for `topic` mode) |

#### Subscribed Topics (topic mode only)

| Topic | Type | Description |
|---|---|---|
| `/velocity_command_node/target_velocities` | `std_msgs/Float64MultiArray` | External velocity commands (7 values, rad/s) |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mode` | string | `"demo"` | Operating mode: `demo` (sine wave), `topic` (external input), `zero` (constant zero) |
| `max_velocity_scale` | double | `0.1` | Fraction of FR3 hardware joint-velocity limits used for clamping (0.0–1.0) |
| `publish_rate` | double | `100.0` | Command publishing frequency [Hz] |
| `demo_amplitude` | double | `0.1` | Sine-wave amplitude for demo mode [rad/s] |
| `demo_frequency` | double | `0.2` | Sine-wave frequency for demo mode [Hz] |

---

### keyboard_teleop_node

**Executable:** `keyboard_teleop_node`  
**Default node name:** `keyboard_teleop_node`

Reads key presses from the terminal and adjusts per-joint velocity commands interactively.

#### Key Bindings

| Key | Action |
|---|---|
| `1` – `7` | Select active joint (J1 … J7) |
| `+` or `=` | Increase active joint velocity by `velocity_step` |
| `-` or `_` | Decrease active joint velocity by `velocity_step` |
| `s` | Stop active joint (velocity → 0) |
| `a` | Stop **all** joints (all velocities → 0) |
| `q` or `ESC` | Quit – sends a zero-velocity stop command first |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/velocity_command_node/target_velocities` | `std_msgs/Float64MultiArray` | Per-joint velocity commands [rad/s] |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `velocity_step` | double | `0.05` | Velocity increment per key press [rad/s] |
| `max_velocity_scale` | double | `0.1` | Fraction of FR3 hardware limits used as the clamp boundary |
| `publish_rate` | double | `50.0` | Key-polling and publishing frequency [Hz] |
| `target_topic` | string | `/velocity_command_node/target_velocities` | Topic to publish commands to |

---

## 6. Launch the Package

### 6.1 Minimal launch (demo sine-wave mode)

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py robot_ip:=172.16.0.2
```

This starts:
- `franka_bringup` (hardware interface + controller_manager)
- `joint_state_broadcaster` + `franka_robot_state_broadcaster`
- `joint_velocity_controller`
- `velocity_command_node` in **demo** mode (gentle sine wave on all joints)
- RViz2 for visualisation

### 6.2 Topic mode (external velocity commands)

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py \
    robot_ip:=172.16.0.2 \
    mode:=topic
```

Then publish commands from another terminal:

```bash
ros2 topic pub /velocity_command_node/target_velocities \
    std_msgs/msg/Float64MultiArray \
    "data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 6.3 With keyboard teleoperation

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py \
    robot_ip:=172.16.0.2 \
    mode:=topic \
    start_teleop:=true
```

A separate **xterm** window opens with the keyboard teleop interface.  
Use `1`–`7` to select a joint, `+`/`-` to adjust its speed.

> **Requires `xterm` installed:** `sudo apt install xterm`

### 6.4 Zero-velocity mode (safe commissioning)

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py \
    robot_ip:=172.16.0.2 \
    mode:=zero
```

All joints receive 0 rad/s continuously – useful for verifying the controller is active without motion.

### 6.5 All launch arguments

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `172.16.0.2` | IP or hostname of the FR3 Control box |
| `load_gripper` | `false` | Attach Franka Hand URDF / gripper driver |
| `mode` | `demo` | `velocity_command_node` operating mode |
| `max_velocity_scale` | `0.1` | Joint velocity limit scale factor (0.0–1.0) |
| `start_teleop` | `false` | Open keyboard teleop in an xterm window |
| `use_rviz` | `true` | Launch RViz2 for joint-state visualisation |

---

## 7. Sending Velocity Commands Manually

Once the stack is running you can send commands from any terminal:

```bash
# Source the workspace
source ~/franka_ros2_ws/install/setup.bash

# Publish a constant command (joint 1 at +0.1 rad/s, others zero)
ros2 topic pub --rate 100 /velocity_command_node/target_velocities \
    std_msgs/msg/Float64MultiArray \
    "data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

Stop all joints immediately:

```bash
ros2 topic pub --once /velocity_command_node/target_velocities \
    std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

Check current joint states:

```bash
ros2 topic echo /joint_states
```

List active controllers:

```bash
ros2 control list_controllers
```

---

## 8. Safety Notes

> ⚠️ **Read before operating the robot.**

1. **Keep `max_velocity_scale` low (≤ 0.1)** while commissioning or developing. The FR3 joints move fast at full speed. At scale 0.1 the maximum commanded velocity is ≈ 0.26 rad/s for joints 1–4.

2. **Keep the robot E-stop button within reach** at all times during operation.

3. **Clear the workspace** – ensure no obstacles are within the robot's reach envelope before enabling any non-zero velocity.

4. **Node crash = zero velocity** – both nodes publish a zero-velocity message on shutdown/`KeyboardInterrupt`, but always keep a finger near the E-stop.

5. **Never exceed 1.0 for `max_velocity_scale`** – the underlying hardware will fault if velocity limits are exceeded.

6. **Real-time priority** – for smooth 1 kHz operation the process must run with elevated RT priority. The `limits.conf` provided in the repository should be applied before running the hardware.

---

## 9. Troubleshooting

### `[ERROR] Could not connect to robot at 172.16.0.2`
- Verify ping works: `ping 172.16.0.2`
- Check that FCI is activated in Desk.
- Check that no other process (e.g., a previous ROS session) is holding the FCI connection.

### Controllers are in `inactive` state
```bash
ros2 control list_controllers
```
Activate manually:
```bash
ros2 control set_controller_state joint_velocity_controller active
```

### `joint_velocity_controller` fails to configure
- Make sure the `franka_hardware` plugin is loaded (not fake hardware).
- Verify joint names in `config/controllers.yaml` match the URDF (`fr3_joint1` … `fr3_joint7`).

### Keyboard teleop window closes immediately
- `xterm` must be installed: `sudo apt install xterm`
- Run the node standalone if preferred:
  ```bash
  ros2 run franka_velocity_ctrl keyboard_teleop_node
  ```

### Robot enters error state / orange light
- Press the E-stop and then release it (twist to unlock).
- In Desk, click **Unlock joints** then **Activate FCI** again.
- In ROS: re-launch the full stack.

---

## License

Apache License 2.0 – see [LICENSE](../LICENSE).
