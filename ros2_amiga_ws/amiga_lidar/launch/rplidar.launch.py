import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port    = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id       = LaunchConfiguration('frame_id')
    scan_mode      = LaunchConfiguration('scan_mode')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port',     default_value='/dev/ttyUSB0',
                              description='RPLidar serial port'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200',
                              description='115200 → A1/A2/A3  |  256000 → S1/S2/S3'),
        DeclareLaunchArgument('frame_id',        default_value='laser_link',
                              description='TF frame yayınlanacak isim'),
        DeclareLaunchArgument('scan_mode',       default_value='Sensitivity',
                              description='RPLidar tarama modu'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'angle_compensate': True,
                'scan_mode':        scan_mode,
            }],
        ),
    ])
