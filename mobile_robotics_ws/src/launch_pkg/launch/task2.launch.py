from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration 
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess

import os

# TASK2
# The students are asked to make the robot able to move
# autonomously in the environment to create a map covering the
# larger possible part of the environment:
# No previous knowledge of the map will be considered during
# the exam;
# Existing packages can be used;
# Implementation (or learning) of (sub)optimal map generation
# processes will be positively evaluated.

def generate_launch_description():
    # Set the environment variable
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Get the path to the launch file we want to include
    included_launch_file_dir1 = get_package_share_directory('turtlebot3_bringup')
    included_launch_file1 = os.path.join(included_launch_file_dir1, 'launch', 'turtlebot3_state_publisher.launch.py')
    included_launch_file_dir2 = get_package_share_directory('turtlebot3_cartographer')
    included_launch_file2 = os.path.join(included_launch_file_dir2, 'launch', 'cartographer.launch.py')
    nav2_launch_file_dir = get_package_share_directory('turtlebot3_navigation2')
    nav2_launch_file = os.path.join(nav2_launch_file_dir, 'launch', 'navigation2.launch.py')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start the state publisher node
    include_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file1),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Start the cartographer node (SLAM)
    include_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file2),
    )

    # Include the Navigation2 launch file
    include_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,'use_rviz': 'false'}.items(),
    )

    my_explorer_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'explorer', 'my_explorer'],
        output='screen',
    )

    return LaunchDescription([
        set_env,
        include_launch1,
        include_launch2,
        include_nav2_launch, 
        my_explorer_node,
    ])



