# velocity_command_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    franka_gazebo_bringup_dir = get_package_share_directory('franka_gazebo_bringup')
    
    # Include the gazebo_franka_arm_example_controller launch with joint_velocity_controller
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_gazebo_bringup_dir, 'launch', 'gazebo_franka_arm_example_controller.launch.py')
        ),
        launch_arguments={
            'controller': 'joint_velocity_controller'
        }.items()
    )
    
    # Velocity command node
    velocity_command_node = Node(
        package='franka_gazebo_velocity_ctrl',  # Replace with your package name
        executable='velocity_command_node',
        name='velocity_command_node',
        output='screen',
        parameters=[{
            # Add any parameters your node needs here
            # 'param1': value1,
            # 'param2': value2,
        }]
    )
    
    return LaunchDescription([
        gazebo_launch,
        velocity_command_node
    ])