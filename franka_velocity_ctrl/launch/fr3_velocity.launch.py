# fr3_velocity.launch.py
# ----------------------
# Launches the Franka Research 3 (FR3) real-hardware stack with a
# JointGroupVelocityController and the franka_velocity_ctrl command node.
#
# Usage:
#   ros2 launch franka_velocity_ctrl fr3_velocity.launch.py robot_ip:=<IP>
#
# Optional arguments:
#   robot_ip            IP / hostname of the FR3           (default: 172.16.0.2)
#   load_gripper        true / false                       (default: false)
#   mode                demo | topic | zero                (default: demo)
#   max_velocity_scale  fraction of FR3 limits [0-1]       (default: 0.1)
#   start_teleop        also launch keyboard teleop        (default: false)
#   use_rviz            launch RViz2 visualisation         (default: true)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_controllers(context, *args, **kwargs):
    """Spawn ros2_control controllers after the controller_manager is ready."""
    return [
        # Step 1 – broadcasters
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'franka_robot_state_broadcaster',
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        ),
        # Step 2 – velocity controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_velocity_controller',
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        ),
    ]


def generate_launch_description():

    # ── Declare launch arguments ──────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='Hostname or IP address of the Franka FR3',
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='false',
        description='Attach Franka Hand gripper (true / false)',
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='demo',
        description='Velocity command mode: demo | topic | zero',
    )
    max_vel_scale_arg = DeclareLaunchArgument(
        'max_velocity_scale',
        default_value='0.1',
        description='Fraction of FR3 hardware joint-velocity limits (0.0–1.0)',
    )
    start_teleop_arg = DeclareLaunchArgument(
        'start_teleop',
        default_value='false',
        description='Also launch the keyboard teleoperation node (true / false)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualisation (true / false)',
    )

    # ── franka_bringup – hardware + controller_manager + robot_state_publisher ─
    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
            )
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': 'false',
            'load_franka_robot_state_broadcaster': 'false',   # we spawn it manually
            'controllers_yaml': PathJoinSubstitution(
                [
                    FindPackageShare('franka_velocity_ctrl'),
                    'config',
                    'controllers.yaml',
                ]
            ),
        }.items(),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config',
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_description'),
                    'rviz',
                    'visualize_franka.rviz',
                ]
            ),
            '-f', 'world',
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    # ── Controller spawner (delayed 3 s to let controller_manager start) ──────
    controller_spawner = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=spawn_controllers)],
    )

    # ── Velocity command node ─────────────────────────────────────────────────
    velocity_node = Node(
        package='franka_velocity_ctrl',
        executable='velocity_command_node',
        name='velocity_command_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'max_velocity_scale': LaunchConfiguration('max_velocity_scale'),
            'publish_rate': 100.0,
        }],
    )

    # ── Keyboard teleop node (optional, opens in xterm) ───────────────────────
    teleop_node = Node(
        package='franka_velocity_ctrl',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'max_velocity_scale': LaunchConfiguration('max_velocity_scale'),
            'target_topic': '/velocity_command_node/target_velocities',
        }],
        condition=IfCondition(LaunchConfiguration('start_teleop')),
    )

    return LaunchDescription([
        robot_ip_arg,
        load_gripper_arg,
        mode_arg,
        max_vel_scale_arg,
        start_teleop_arg,
        use_rviz_arg,
        franka_bringup,
        rviz_node,
        controller_spawner,
        velocity_node,
        teleop_node,
    ])
