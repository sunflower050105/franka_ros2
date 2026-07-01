# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_controller(context: LaunchContext, controller_name):
    controller_name_str = context.perform_substitution(controller_name)
    return [Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            controller_name_str,
            '--controller-manager-timeout', '30',
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_gazebo_bringup'),
            'config',
            'franka_gazebo_controllers.yaml'
        ])],
        output='screen',
    )]


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand,
                          cam1_focal_length=None, cam2_focal_length=None,
                          cam1_sensor_width=None, cam2_sensor_width=None,
                          cam_width=None, cam_height=None, cam_rate=None,
                          cam_clip_near=None, cam_clip_far=None):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf',
        'franka_arm.gazebo.xacro'
    )

    mappings = {
        'robot_type': robot_type_str,
        'hand': load_gripper_str,
        'gazebo': 'true',
        'ee_id': franka_hand_str,
        'gazebo_effort': 'true',
    }

    # Forward optional camera parameters when provided
    def _add(key, lc_sub):
        if lc_sub is not None:
            mappings[key] = context.perform_substitution(lc_sub)

    _add('cam1_focal_length', cam1_focal_length)
    _add('cam2_focal_length', cam2_focal_length)
    _add('cam1_sensor_width', cam1_sensor_width)
    _add('cam2_sensor_width', cam2_sensor_width)
    _add('cam_width',         cam_width)
    _add('cam_height',        cam_height)
    _add('cam_rate',          cam_rate)
    _add('cam_clip_near',     cam_clip_near)
    _add('cam_clip_far',      cam_clip_far)

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings=mappings
    )

    if not isinstance(robot_description_config, xml.dom.minidom.Document):
        raise RuntimeError(
            f'The given xacro file {franka_xacro_file} is not a valid xml format.')

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'
    controller_name = 'controller'
    rviz_name = 'rviz'
    gz_args_name = 'gz_args'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)
    controller = LaunchConfiguration(controller_name)
    rviz = LaunchConfiguration(rviz_name)
    gz_args = LaunchConfiguration(gz_args_name)

    # Camera LaunchConfigurations
    cam1_focal_length = LaunchConfiguration('cam1_focal_length')
    cam2_focal_length = LaunchConfiguration('cam2_focal_length')
    cam1_sensor_width = LaunchConfiguration('cam1_sensor_width')
    cam2_sensor_width = LaunchConfiguration('cam2_sensor_width')
    cam_width         = LaunchConfiguration('cam_width')
    cam_height        = LaunchConfiguration('cam_height')
    cam_rate          = LaunchConfiguration('cam_rate')
    cam_clip_near     = LaunchConfiguration('cam_clip_near')
    cam_clip_far      = LaunchConfiguration('cam_clip_far')

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    controller_launch_argument = DeclareLaunchArgument(
        controller_name,
        default_value='gravity_compensation_example_controller',
        description='The controller name to be used. You can choose one from the franka_example_controllers.')
    gz_args_launch_argument = DeclareLaunchArgument(
        gz_args_name,
        default_value='-r empty.sdf',
        description='Extra args to be forwared to gazebo')
    rviz_launch_argument = DeclareLaunchArgument(
        rviz_name,
        default_value='true',
        description='true/false for visualizing the robot in rviz')

    # Camera arguments (declared here so IncludeLaunchDescription can forward them)
    cam1_fl_arg = DeclareLaunchArgument(
        'cam1_focal_length', default_value='3.5',
        description='Camera-1 (wrist) focal length in mm')
    cam2_fl_arg = DeclareLaunchArgument(
        'cam2_focal_length', default_value='8.0',
        description='Camera-2 (elbow) focal length in mm')
    cam1_sw_arg = DeclareLaunchArgument(
        'cam1_sensor_width', default_value='3.68',
        description='Camera-1 sensor width in mm (1/4" CMOS = 3.68 mm)')
    cam2_sw_arg = DeclareLaunchArgument(
        'cam2_sensor_width', default_value='3.68',
        description='Camera-2 sensor width in mm (1/4" CMOS = 3.68 mm)')
    cam_width_arg = DeclareLaunchArgument(
        'cam_width', default_value='640',
        description='Camera image width in pixels')
    cam_height_arg = DeclareLaunchArgument(
        'cam_height', default_value='480',
        description='Camera image height in pixels')
    cam_rate_arg = DeclareLaunchArgument(
        'cam_rate', default_value='30',
        description='Camera frame rate in Hz')
    cam_clip_near_arg = DeclareLaunchArgument(
        'cam_clip_near', default_value='0.05',
        description='Near clipping plane in metres')
    cam_clip_far_arg = DeclareLaunchArgument(
        'cam_clip_far', default_value='20.0',
        description='Far clipping plane in metres')

    # Get robot description  (camera args forwarded to xacro)
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, franka_hand,
              cam1_focal_length, cam2_focal_length,
              cam1_sensor_width, cam2_sensor_width,
              cam_width, cam_height, cam_rate,
              cam_clip_near, cam_clip_far])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    gazebo_empty_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     namespace=namespace,
                     arguments=['--display-config', rviz_file, '-f', 'world'],
                     condition=IfCondition(rviz))

    launch_controller = OpaqueFunction(
        function=load_controller,
        args=[controller]
    )

    return LaunchDescription([
        # Standard arguments
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        robot_type_launch_argument,
        namespace_launch_argument,
        controller_launch_argument,
        gz_args_launch_argument,
        rviz_launch_argument,
        # Camera arguments
        cam1_fl_arg,
        cam2_fl_arg,
        cam1_sw_arg,
        cam2_sw_arg,
        cam_width_arg,
        cam_height_arg,
        cam_rate_arg,
        cam_clip_near_arg,
        cam_clip_far_arg,
        # Launch items
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[launch_controller],
            )
        ),
    ])
