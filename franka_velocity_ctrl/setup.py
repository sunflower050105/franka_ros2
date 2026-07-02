from setuptools import find_packages, setup

package_name = 'franka_velocity_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/controllers.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/fr3_velocity.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Joint velocity control for the Franka Research 3 real hardware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_command_node = franka_velocity_ctrl.velocity_command_node:main',
            'keyboard_teleop_node  = franka_velocity_ctrl.keyboard_teleop_node:main',
            'example_joint_vel_node = franka_velocity_ctrl.example_joint_vel_node:main'
        ],
    },
)
