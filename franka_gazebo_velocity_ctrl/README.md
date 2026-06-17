# franka_gazebo_velocity_ctrl

A small ROS 2 Python package providing a velocity command node for the Franka Gazebo robot simulation.

The package starts the Gazebo Franka launch with a `joint_velocity_controller` and publishes a sample `Float64MultiArray` velocity command to `/joint_velocity_controller/commands` at 100 Hz.

## Package contents

- `franka_gazebo_velocity_ctrl/velocity_command_node.py` - ROS 2 node publishing joint velocity commands.
- `launch/fr_gazebo_velocity.launch.py` - Launch file that includes `franka_gazebo_bringup` Gazebo launch and the velocity command node.
- `setup.py` - Package setup configuration.
- `package.xml` - ROS 2 package manifest.

## Dependencies

- `rclpy`
- `std_msgs`
- `controller_manager`
- `velocity_controllers`
- `joint_state_broadcaster`
- `gazebo_ros`
- `franka_description`

## Usage

1. Build the workspace from the ROS 2 workspace root:

```bash
colcon build --packages-select franka_gazebo_velocity_ctrl
```

2. Source the install setup file:

```bash
. install/setup.sh
```

3. Launch the package:

```bash
ros2 launch franka_gazebo_velocity_ctrl fr_gazebo_velocity.launch.py
```

This will launch the Gazebo Franka simulation with the `joint_velocity_controller` and start the velocity command node.

## Node details

- Node name: `velocity_command_node`
- Topic published: `/joint_velocity_controller/commands`
- Message type: `std_msgs/msg/Float64MultiArray`
- Frequency: `100 Hz`

## Customization

To change the published velocity profile, edit `franka_gazebo_velocity_ctrl/velocity_command_node.py`.

## Notes

- The current command generator provides a simple example waveform for joints 4 and 5.
- Ensure the Gazebo controller configuration supports `joint_velocity_controller` before launching.
