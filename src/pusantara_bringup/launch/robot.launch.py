from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vectra_odometry'),
                'launch',
                'odometry.launch.py'
            )
        )
    )

    return LaunchDescription([

        ##################################################
        # LiDAR Driver
        ##################################################

        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            output='screen',
            parameters=[
                {
                    'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                    'frame_id': 'laser'
                }
            ]
        ),

        ##################################################
        # IMU
        ##################################################

        Node(
            package='bno055_driver',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        ##################################################
        # Motion Manager
        ##################################################

        Node(
            package='motion_manager',
            executable='motion_manager',
            name='motion_manager',
            output='screen'
        ),

        ##################################################
        # Serial Bridge
        ##################################################

        Node(
            package='serial_bridge',
            executable='bridge_node',
            name='serial_bridge',
            output='screen'
        ),
        
        
        ##################################################
        # cmd_vel_to_robot_command
        ##################################################

        Node(
            package='cmd_vel_bridge',
            executable='cmd_vel_to_robot_command',
            name='cmd_vel_bridge',
            output='screen'
        ),

        ##################################################
        # Odometry
        ##################################################

        odom,

        ##################################################
        # Laser TF
        ##################################################

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.135',
                '0.0',
                '0.60',
                '0',
                '0',
                '0',
                'base_link',
                'laser'
            ],
            name='laser_tf',
            output='screen'
        ),
    ])
