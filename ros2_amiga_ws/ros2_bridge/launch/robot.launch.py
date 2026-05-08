import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lidar  = get_package_share_directory('amiga_lidar')
    pkg_ekf    = get_package_share_directory('amiga_navsat_ekf')
    pkg_nav    = get_package_share_directory('amiga_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'lidar.launch.py')
        ),
    )

    # EKF + navsat_transform: fromLL servisi burada ayağa kalkar
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ekf, 'launch', 'navsat_ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Nav2: EKF hazır olduktan sonra başlasın
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ],
    )

    # task_manager: Nav2 hazır olduktan sonra başlasın
    task_manager_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='task_manager',
                executable='task_manager_node',
                name='task_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        lidar_launch,
        navsat_launch,
        navigation_launch,
        task_manager_node,
    ])
