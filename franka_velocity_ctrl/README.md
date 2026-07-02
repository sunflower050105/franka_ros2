# franka_velocity_ctrl

A ROS 2 package for **joint-velocity control** of the **Franka Research 3 (FR3)** robotic arm on real hardware.

It bridges the gap between a high-level planner and the Franka hardware: it handles collision-behavior relaxation, real-time velocity smoothing, and convenient operating modes so you can focus on what you want the robot to do rather than how to talk to it safely.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Prerequisites](#2-prerequisites)
3. [Hardware – Connecting the FR3](#3-hardware--connecting-the-fr3)
4. [Workspace Setup & Build](#4-workspace-setup--build)
5. [Package Structure](#5-package-structure)
6. [Nodes Reference](#6-nodes-reference)
   - [velocity_command_node](#velocity_command_node)
   - [keyboard_teleop_node](#keyboard_teleop_node)
7. [The C++ Hardware Controller (JointVelocityExampleController)](#7-the-c-hardware-controller-jointvelocityexamplecontroller)
8. [Launch the Package](#8-launch-the-package)
9. [Sending Velocity Commands Manually](#9-sending-velocity-commands-manually)
10. [Customization & Extension Guide](#10-customization--extension-guide)
11. [Tuning the Velocity Filter](#11-tuning-the-velocity-filter)
12. [Safety Notes](#12-safety-notes)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Architecture

### Overview

The stack is organized in three layers:

```
┌─────────────────────────────────────────────────────────────────┐
│                      Your Application / Planner                  │
│    (publishes Float64MultiArray to ~/target_velocities)          │
└───────────────────────────────┬─────────────────────────────────┘
                                │  std_msgs/Float64MultiArray
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│             velocity_command_node  (Python, this pkg)            │
│  • mode: demo | topic | zero                                    │
│  • clamps each joint to max_velocity_scale × hardware limit     │
│  • publishes at 100 Hz to the controller command topic          │
└───────────────────────────────┬─────────────────────────────────┘
                                │  std_msgs/Float64MultiArray
                                │  /joint_velocity_example_controller/commands
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│   JointVelocityExampleController  (C++, franka_example_controllers)  │
│  • runs at 1 kHz inside ros2_control                            │
│  • exponential low-pass filter:  v_out += α(v_target − v_out)  │
│  • eliminates acceleration discontinuities → no reflex          │
│  • calls set_full_collision_behavior on configure               │
└───────────────────────────────┬─────────────────────────────────┘
                                │  /velocity  command interfaces
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│              FrankaHardwareInterface  (franka_hardware)          │
│  • libfranka real-time control loop at 1 kHz                    │
│  • communicates with FR3 Control box over Ethernet              │
└─────────────────────────────────────────────────────────────────┘
```

### Why JointVelocityExampleController instead of a generic controller?

Franka's firmware enforces safety reflexes.  A plain `JointGroupVelocityController` passes commands straight through and can trigger the reflex `communication_constraints_violation` because it does **not** call `set_full_collision_behavior` to relax the thresholds.  `JointVelocityExampleController` calls that service automatically during `on_configure()`.

### Why the exponential low-pass filter?

Even after relaxing collision thresholds, the reflex `joint_motion_generator_acceleration_discontinuity` fires when the **acceleration** itself contains a step (i.e. non-zero jerk).  A hard clamp on Δv/cycle creates exactly such a step.  An exponential filter:

```
v_out[k] = v_out[k-1] + α × (v_target − v_out[k-1])
```

produces an acceleration that **starts at its maximum and decays smoothly**. There is no instantaneous acceleration jump, so the reflex never fires.

### Data-flow diagram

```
keyboard_teleop_node ──┐
                       │ Float64MultiArray
external planner ──────┤  (7 × rad/s)
                       │
                       ▼
             velocity_command_node
             ┌──────────────────────┐
             │ mode selector        │
             │ velocity clamp       │  → /joint_velocity_example_controller/commands
             │ 100 Hz publish timer │
             └──────────────────────┘
                       │
                       ▼
       JointVelocityExampleController (1 kHz)
       ┌──────────────────────────────────────┐
       │ realtime command buffer              │
       │ exponential LPF per joint            │
       │ set_value() → hardware interfaces    │
       └──────────────────────────────────────┘
                       │
                       ▼
           FrankaHardwareInterface / libfranka
```

### Active controllers when running

```
Name                           Type                                              State
─────────────────────────────────────────────────────────────────────────────────────
joint_state_broadcaster        joint_state_broadcaster/JointStateBroadcaster    active
franka_robot_state_broadcaster franka_robot_state_broadcaster/...               active
joint_velocity_example_controller franka_example_controllers/...                active
```

---

## 2. Prerequisites

### Software

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 LTS (Noble) |
| ROS 2 | Jazzy |
| `franka_ros2` | ≥ 0.1.15 (this workspace) |
| `ros2_control` / `ros2_controllers` | Jazzy |
| Python | 3.10+ |

Install ROS 2 controllers if not already present:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-velocity-controllers \
                 ros-jazzy-joint-state-broadcaster \
                 ros-jazzy-controller-manager \
                 xterm       # required for keyboard teleop in launch
```

### FR3 Firmware

Make sure the Franka Research 3 is running **FCI-compatible firmware** (≥ 5.x).  
Check the [Franka World](https://franka.world/) or your robot's admin panel for the firmware version.

---

## 3. Hardware – Connecting the FR3

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
2. Log in (default: `franka` / `franka123`).
3. Click **Activate FCI** (or release the brakes button, depending on firmware).
4. Make sure the status light on the robot turns **blue** (Ready / FCI active).

> ⚠️ The robot **must be in FCI mode** before launching any ROS 2 controller.


## 4. Workspace Setup & Build

```bash
# 1. Source ROS 2 base
source /opt/ros/jazzy/setup.bash

# 2. Install rosdep dependencies (run once from workspace root)
cd ~/franka_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build only the packages you changed
colcon build --packages-select franka_example_controllers franka_velocity_ctrl \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Source the overlay
source install/setup.bash
```

> **Symlink install tip:** Add `--symlink-install` if you are actively editing Python files — changes take effect without rebuilding.

---

## 5. Package Structure

```
franka_velocity_ctrl/
├── config/
│   └── controllers.yaml              # ros2_control controller parameters + filter tuning
├── franka_velocity_ctrl/
│   ├── __init__.py
│   ├── velocity_command_node.py      # main velocity command publisher (3 modes)
│   └── keyboard_teleop_node.py       # interactive keyboard joint velocity teleop
├── launch/
│   └── fr3_velocity.launch.py        # top-level launch file
├── resource/
│   └── franka_velocity_ctrl          # ament resource index marker
├── package.xml
├── setup.cfg
├── setup.py
└── README.md

# Key files in related packages:
franka_example_controllers/
├── include/franka_example_controllers/fr3/
│   └── joint_velocity_example_controller.hpp    # controller class declaration
└── src/fr3/
    └── joint_velocity_example_controller.cpp    # update(), on_activate(), filter logic
```

---

## 6. Nodes Reference

### velocity_command_node

**Executable:** `velocity_command_node`  
**Default node name:** `velocity_command_node`  
**Source:** `franka_velocity_ctrl/velocity_command_node.py`

This node acts as the **bridge between your planner and the hardware controller**.  
It selects a velocity source (demo/topic/zero), clamps each joint to the configured limit, and republishes at a fixed rate.

#### Operating Modes

| Mode | Behaviour |
|---|---|
| `demo` | Node publishes nothing; the C++ controller's built-in sine-wave demo runs |
| `topic` | Forwards commands received on `~/target_velocities` to the hardware |
| `zero` | Continuously publishes `[0, 0, 0, 0, 0, 0, 0]` — safe commissioning mode |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/joint_velocity_example_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands sent to the hardware controller [rad/s] |
| `~/target_velocities` | `std_msgs/Float64MultiArray` | Echo of the internally computed target (also the subscription target in `topic` mode) |

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `~/target_velocities` | `std_msgs/Float64MultiArray` | External velocity commands in `topic` mode (7 values, rad/s) |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mode` | string | `"demo"` | Operating mode: `demo`, `topic`, or `zero` |
| `max_velocity_scale` | double | `0.1` | Fraction of FR3 hardware joint-velocity limits for clamping (0.0–1.0) |
| `publish_rate` | double | `100.0` | Command publishing frequency [Hz] |
| `demo_amplitude` | double | `0.1` | Sine-wave amplitude for demo mode [rad/s] |
| `demo_frequency` | double | `0.2` | Sine-wave frequency for demo mode [Hz] |
| `command_topic` | string | `/joint_velocity_example_controller/commands` | Downstream controller command topic |

#### FR3 Hardware Joint Velocity Limits

| Joint | Limit [rad/s] | At scale 0.1 |
|---|---|---|
| J1 | 2.62 | 0.262 |
| J2 | 2.62 | 0.262 |
| J3 | 2.62 | 0.262 |
| J4 | 2.62 | 0.262 |
| J5 | 5.26 | 0.526 |
| J6 | 4.18 | 0.418 |
| J7 | 5.26 | 0.526 |

---

### keyboard_teleop_node

**Executable:** `keyboard_teleop_node`  
**Default node name:** `keyboard_teleop_node`  
**Source:** `franka_velocity_ctrl/keyboard_teleop_node.py`

Reads single-character key presses from a terminal and adjusts per-joint velocity commands interactively.  Must be used together with `velocity_command_node` in `topic` mode.

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

## 7. The C++ Hardware Controller (JointVelocityExampleController)

**Package:** `franka_example_controllers`  
**Plugin type:** `franka_example_controllers/JointVelocityExampleController`  
**Control loop rate:** 1 kHz (driven by `ros2_control`)

This controller runs inside the `ros2_control_node` real-time loop. It is responsible for:

1. **Calling `set_full_collision_behavior`** during `on_configure()` to relax Franka's default reflex thresholds so velocity control is permitted.
2. **Reading commands** from a `RealtimeBuffer` (filled by the ROS subscription at non-RT priority).
3. **Filtering** each joint's commanded velocity through an exponential low-pass filter to avoid acceleration discontinuities.
4. **Writing** the smoothed velocity to each joint's `command_interface`.

### Lifecycle sequence

```
on_init()        → declares parameters (arm_prefix, gazebo, filter_coefficient)
on_configure()   → calls set_full_collision_behavior, creates subscriber
on_activate()    → seeds filter state from measured joint velocities
update() × 1kHz → LPF step → set_value() per joint
on_deactivate()  → (not overridden; ros2_control sends zero on deactivation)
```

### Controller Parameters (set in `config/controllers.yaml`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `arm_prefix` | string | `""` | Prefix added to joint names (for multi-arm setups) |
| `gazebo` | bool | `false` | Skip `set_full_collision_behavior` call when using Gazebo |
| `filter_coefficient` | double | `0.02` | Exponential LPF coefficient α ∈ (0, 1] |

---

## 8. Launch the Package

### 8.1 Minimal launch (demo sine-wave mode)

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py robot_ip:=172.16.0.2
```

This starts:
- `franka_bringup` (hardware interface + controller_manager + robot_state_publisher)
- `joint_state_broadcaster` (spawned by `franka_bringup`)
- `franka_robot_state_broadcaster` (spawned by this launch file)
- `joint_velocity_example_controller`
- `velocity_command_node` in **demo** mode (no commands published; controller uses built-in sine wave)
- RViz2 for visualisation

### 8.2 Topic mode (external velocity commands)

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

### 8.3 With keyboard teleoperation

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py \
    robot_ip:=172.16.0.2 \
    mode:=topic \
    start_teleop:=true
```

A separate **xterm** window opens with the keyboard teleop interface.  
Use `1`–`7` to select a joint, `+`/`-` to adjust its speed, `a` to stop all.

> **Requires `xterm` installed:** `sudo apt install xterm`

### 8.4 Zero-velocity mode (safe commissioning)

```bash
ros2 launch franka_velocity_ctrl fr3_velocity.launch.py \
    robot_ip:=172.16.0.2 \
    mode:=zero
```

All joints receive 0 rad/s continuously – useful for verifying the controller is active without motion.

### 8.5 All launch arguments

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `172.16.0.2` | IP or hostname of the FR3 Control box |
| `load_gripper` | `false` | Attach Franka Hand URDF / gripper driver |
| `mode` | `demo` | `velocity_command_node` operating mode |
| `max_velocity_scale` | `0.1` | Joint velocity limit scale factor (0.0–1.0) |
| `start_teleop` | `false` | Open keyboard teleop in an xterm window |
| `use_rviz` | `true` | Launch RViz2 for joint-state visualisation |

---

## 9. Sending Velocity Commands Manually

Once the stack is running in `topic` mode you can send commands from any terminal:

```bash
source ~/franka_ros2_ws/install/setup.bash

# Move joint 1 at +0.05 rad/s (others stationary)
ros2 topic pub /velocity_command_node/target_velocities \
    std_msgs/msg/Float64MultiArray \
    "data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Stop all joints
ros2 topic pub --once /velocity_command_node/target_velocities \
    std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

Useful monitoring commands:

```bash
# Check controller states
ros2 control list_controllers

# Monitor actual joint states
ros2 topic echo /joint_states

# Echo what the controller is receiving
ros2 topic echo /joint_velocity_example_controller/commands

# Check publishing rate
ros2 topic hz /joint_velocity_example_controller/commands
```

---

## 10. Customization & Extension Guide

### 10.1 Adding a new operating mode to velocity_command_node

Open `franka_velocity_ctrl/velocity_command_node.py`.

1. Add your mode name to the `valid_modes` tuple:
   ```python
   valid_modes = ('demo', 'topic', 'zero', 'my_mode')
   ```

2. Add a branch in `_compute_velocities()`:
   ```python
   if self._mode == 'my_mode':
       # e.g. read from a shared variable updated by a subscriber
       return list(self._my_custom_velocities)
   ```

3. Add any subscriber or timer logic in `__init__()`.

### 10.2 Connecting a custom planner or MoveIt trajectory

In `topic` mode the velocity_command_node simply relays whatever lands on `~/target_velocities`.  Your planner just needs to publish `std_msgs/Float64MultiArray` with 7 values to that topic at any rate up to 100 Hz (the node's own timer handles the 100 Hz forwarding).

```python
# Minimal Python planner snippet
from std_msgs.msg import Float64MultiArray
pub = node.create_publisher(Float64MultiArray, '/velocity_command_node/target_velocities', 10)
msg = Float64MultiArray()
msg.data = [v_j1, v_j2, v_j3, v_j4, v_j5, v_j6, v_j7]
pub.publish(msg)
```

### 10.3 Bypassing velocity_command_node entirely

If your planner runs at 100 Hz or faster and you want to skip the intermediate node, publish directly to the controller's command topic:

```bash
ros2 topic pub /joint_velocity_example_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "data: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

The C++ controller reads from this topic directly via its internal subscriber.

### 10.4 Adjusting velocity limits per-joint

Limits are defined as a constant list in the Python node:

```python
# franka_velocity_ctrl/velocity_command_node.py
FR3_MAX_JOINT_VELOCITIES = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]
```

Scale is applied at runtime via `max_velocity_scale`.  
To set per-joint scales, you could replace the scalar with a list parameter:

```python
self.declare_parameter('per_joint_scale', [0.1] * 7)
scales = self.get_parameter('per_joint_scale').get_parameter_value().double_array_value
self._limits = [v * s for v, s in zip(FR3_MAX_JOINT_VELOCITIES, scales)]
```

### 10.5 Changing the controller update rate

The controller runs at whatever rate `controller_manager` is configured for.  
In `config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000   # Hz — must match Franka's 1 kHz loop
```

> ⚠️ Do **not** lower this below 1000 Hz for real hardware. The Franka firmware expects commands at 1 kHz. Missing cycles causes `communication_constraints_violation`.

### 10.6 Replacing the C++ controller with a custom one

If you need more complex control (e.g. Cartesian velocity, impedance), you can swap the controller:

1. Create your controller in a new package, deriving from `controller_interface::ControllerInterface`.
2. Make sure `on_configure()` calls `set_full_collision_behavior`.
3. Implement the same exponential filter (or a more sophisticated trajectory generator) in `update()`.
4. Register it with `pluginlib` and update `controllers.yaml`:
   ```yaml
   joint_velocity_example_controller:
     type: my_package/MyCustomController
   ```
5. Update `fr3_velocity.launch.py` `spawn_controllers()` if the controller name changes.

### 10.7 Using with a namespace (multi-robot setups)

The `/**` wildcard in `controllers.yaml` makes parameters namespace-agnostic.  
Pass a namespace in the launch file if needed:

```python
Node(
    package='franka_velocity_ctrl',
    executable='velocity_command_node',
    namespace='robot_a',
    ...
)
```

And update `command_topic` parameter to match:
```
/robot_a/joint_velocity_example_controller/commands
```

---

## 11. Tuning the Velocity Filter

The exponential low-pass filter in `JointVelocityExampleController` is the key to avoiding Franka's acceleration-discontinuity reflex.  Tune `filter_coefficient` in `config/controllers.yaml`:

```yaml
joint_velocity_example_controller:
  ros__parameters:
    filter_coefficient: 0.02   # α ∈ (0, 1]
```

### How to choose α

At 1 kHz the peak acceleration for a step command of |Δv| rad/s is:

```
peak_accel ≈ α × |Δv| / 0.001   [rad/s²]
```

FR3 joints tolerate approximately **10 rad/s²** before the reflex fires.  Use the table below as a starting point:

| α | Time constant (τ) | Peak accel for Δv = 0.1 rad/s | Peak accel for Δv = 0.5 rad/s |
|---|---|---|---|
| 0.005 | 200 ms | 0.5 rad/s² | 2.5 rad/s² |
| **0.02** (default) | **50 ms** | **2.0 rad/s²** | **10 rad/s²** |
| 0.05 | 20 ms | 5.0 rad/s² | 25 rad/s² ⚠️ |
| 0.10 | 10 ms | 10.0 rad/s² ⚠️ | reflex likely |

**Recommendation:**
- Start with `α = 0.02` (default).
- If the robot responds too sluggishly, increase to `0.03` or `0.04`, but keep `|Δv| × α / 0.001 < 8 rad/s²` for margin.
- If commanding large velocity steps (> 0.3 rad/s) reduce α proportionally.

The time constant τ gives you the 63% rise time: for τ = 50 ms and a 0.1 rad/s step, you reach 0.063 rad/s after 50 ms.

---

## 12. Safety Notes

> ⚠️ **Read before operating the robot.**

1. **Keep `max_velocity_scale` low (≤ 0.1)** while commissioning or developing. The FR3 joints move fast at full speed. At scale 0.1 the maximum commanded velocity is ≈ 0.26 rad/s for joints 1–4.

2. **Keep the robot E-stop button within reach** at all times during operation.

3. **Clear the workspace** – ensure no obstacles are within the robot's reach envelope before enabling any non-zero velocity.

4. **Node crash = zero velocity** – both nodes publish a zero-velocity message on shutdown/`KeyboardInterrupt`, but always keep a finger near the E-stop.

5. **Never exceed 1.0 for `max_velocity_scale`** – the underlying hardware will fault if velocity limits are exceeded.

6. **Keep `filter_coefficient` ≤ 0.04** for typical velocity commands to stay within the FR3 acceleration limit of ~10 rad/s². See the tuning table in §11.

7. **Real-time priority** – for smooth 1 kHz operation the process must run with elevated RT priority. Apply the `limits.conf` provided in the repository and add your user to the `realtime` group before running the hardware.

8. **`overruns.manage: false`** is set in `controllers.yaml` intentionally. Enabling overrun management would cause `ros2_control` to deactivate controllers when the laptop misses a 1 ms deadline, which then triggers the `communication_constraints_violation` reflex. The Franka firmware itself tolerates a small number of missed cycles; it is safer to leave the controller alive.

---

## 13. Troubleshooting

### `[ERROR] Could not connect to robot at 172.16.0.2`
- Verify ping works: `ping 172.16.0.2`
- Check that FCI is activated in Desk.
- Check that no other process (e.g., a previous ROS session) is holding the FCI connection.

### Controllers are in `inactive` state
```bash
ros2 control list_controllers
# Manually activate if needed:
ros2 control set_controller_state joint_velocity_example_controller active
```

### `joint_motion_generator_acceleration_discontinuity` reflex / controllers killed
- Lower `filter_coefficient` in `config/controllers.yaml` (try `0.01`).
- Make sure your command step size is not too large: `|Δv| × α / 0.001 < 8 rad/s²`.
- Verify the built workspace is installed: `source ~/franka_ros2_ws/install/setup.bash`.

### `communication_constraints_violation` reflex
- Verify `overruns.manage: false` is set in `controllers.yaml`.
- Check that you are running a real-time kernel and the process has elevated RT priority.
- Restart with a clean launch — never hot-reload the hardware interface.

### `joint_velocity_example_controller` fails to configure
- Make sure `franka_hardware` plugin is loaded (not fake hardware).
- Verify joint names in `config/controllers.yaml` match the URDF (`fr3_joint1` … `fr3_joint7`).
- Check that `service_server/set_full_collision_behavior` is available:
  ```bash
  ros2 service list | grep collision
  ```

### Keyboard teleop window closes immediately
- `xterm` must be installed: `sudo apt install xterm`
- Run the node standalone if preferred:
  ```bash
  ros2 run franka_velocity_ctrl keyboard_teleop_node
  ```

### Robot enters error state / orange light
1. Press the E-stop and then release it (twist to unlock).
2. In Desk, click **Unlock joints** then **Activate FCI** again.
3. In ROS: re-launch the full stack.

### Checking what velocity the controller is actually sending

Enable DEBUG logging for the controller:
```bash
ros2 run rclcpp_components component_container --ros-args \
    --log-level joint_velocity_example_controller:=debug
```
Or set it at launch: add `'--log-level'` arguments to the `ros2_control_node`.

---

## License

Apache License 2.0 – see [LICENSE](../LICENSE).
