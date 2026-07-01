# fr_gazebo_velocity.launch.py
#
# Launches the FR3 arm in Gazebo with:
#   - joint_velocity_controller
#   - velocity_command_node
#   - TWO cameras (wrist + elbow) whose focal lengths are configurable
#   - ros_gz_bridge for camera image topics → ROS 2
#
# Camera arguments
# ----------------
#   cam1_focal_length  (mm) – wrist camera  (default: 3.5 mm wide-angle)
#   cam2_focal_length  (mm) – elbow camera  (default: 8.0 mm telephoto)
#   cam1_sensor_width  (mm) – sensor width for cam1 (default: 3.68 mm  = 1/4" CMOS)
#   cam2_sensor_width  (mm) – sensor width for cam2 (default: 3.68 mm  = 1/4" CMOS)
#
# Usage
# -----
#   ros2 launch franka_gazebo_velocity_ctrl fr_gazebo_velocity.launch.py
#
#   # Custom focal lengths
#   ros2 launch franka_gazebo_velocity_ctrl fr_gazebo_velocity.launch.py \
#       cam1_focal_length:=6.0 cam2_focal_length:=12.0

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ------------------------------------------------------------------
    # Camera focal-length / sensor arguments
    # ------------------------------------------------------------------
    cam1_fl_arg = DeclareLaunchArgument(
        'cam1_focal_length', default_value='3.5',
        description='Wrist-camera (cam1) focal length in mm. '
                    'Shorter value = wider FOV; longer value = narrower FOV.')

    cam2_fl_arg = DeclareLaunchArgument(
        'cam2_focal_length', default_value='8.0',
        description='Elbow-camera (cam2) focal length in mm. '
                    'Shorter value = wider FOV; longer value = narrower FOV.')

    cam1_sw_arg = DeclareLaunchArgument(
        'cam1_sensor_width', default_value='3.68',
        description='Wrist-camera sensor width in mm (1/4" CMOS = 3.68 mm)')

    cam2_sw_arg = DeclareLaunchArgument(
        'cam2_sensor_width', default_value='3.68',
        description='Elbow-camera sensor width in mm (1/4" CMOS = 3.68 mm)')

    cam_width_arg = DeclareLaunchArgument(
        'cam_width', default_value='1280',
        description='Camera image width in pixels')

    cam_height_arg = DeclareLaunchArgument(
        'cam_height', default_value='960',
        description='Camera image height in pixels')

    cam_rate_arg = DeclareLaunchArgument(
        'cam_rate', default_value='30',
        description='Camera frame rate in Hz')

    # ------------------------------------------------------------------
    # LaunchConfiguration references
    # ------------------------------------------------------------------
    cam1_fl     = LaunchConfiguration('cam1_focal_length')
    cam2_fl     = LaunchConfiguration('cam2_focal_length')
    cam1_sw     = LaunchConfiguration('cam1_sensor_width')
    cam2_sw     = LaunchConfiguration('cam2_sensor_width')
    cam_width   = LaunchConfiguration('cam_width')
    cam_height  = LaunchConfiguration('cam_height')
    cam_rate    = LaunchConfiguration('cam_rate')

    # ------------------------------------------------------------------
    # Include the base Gazebo launch (with the camera xacro args forwarded)
    # ------------------------------------------------------------------
    franka_gazebo_bringup_dir = get_package_share_directory('franka_gazebo_bringup')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_gazebo_bringup_dir, 'launch',
                         'gazebo_franka_arm_example_controller.launch.py')
        ),
        launch_arguments={
            'controller':         'joint_velocity_controller',
            # Forward camera parameters so the xacro can compute the correct HFOV
            'cam1_focal_length':  cam1_fl,
            'cam2_focal_length':  cam2_fl,
            'cam1_sensor_width':  cam1_sw,
            'cam2_sensor_width':  cam2_sw,
            'cam_width':          cam_width,
            'cam_height':         cam_height,
            'cam_rate':           cam_rate,
        }.items()
    )

    # ------------------------------------------------------------------
    # ros_gz_bridge  – camera topics  (Gazebo → ROS 2)
    #
    # Gazebo publishes on:   /fr3_camera1/image_raw   /fr3_camera2/image_raw
    # Bridge re-publishes on: /camera1/image_raw      /camera2/image_raw
    # ------------------------------------------------------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            # Camera 1 – wrist camera
            '/fr3_camera1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/fr3_camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Camera 2 – elbow camera
            '/fr3_camera2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/fr3_camera2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/fr3_camera1/image_raw',   '/camera1/image_raw'),
            ('/fr3_camera1/camera_info', '/camera1/camera_info'),
            ('/fr3_camera2/image_raw',   '/camera2/image_raw'),
            ('/fr3_camera2/camera_info', '/camera2/camera_info'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Velocity command node
    # ------------------------------------------------------------------
    velocity_command_node = Node(
        package='franka_gazebo_velocity_ctrl',
        executable='velocity_command_node',
        name='velocity_command_node',
        output='screen',
        parameters=[{}]
    )

    # ------------------------------------------------------------------
    return LaunchDescription([
        # Camera arguments
        cam1_fl_arg,
        cam2_fl_arg,
        cam1_sw_arg,
        cam2_sw_arg,
        cam_width_arg,
        cam_height_arg,
        cam_rate_arg,
        # Launch items
        gazebo_launch,
        camera_bridge,
        velocity_command_node,
    ])
