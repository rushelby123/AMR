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
# After the mapping of the scenario (Task 2), the students are asked
# to make the robot able to navigate in the environment reaching a
# set of goals contained in a text file:
# The alignement between the map and the navigation system
# must be performed;
# Implementation (or learning) of (sub)optimal map alignement
# processes will be positively evaluated.

def generate_launch_description():
    # Set the environment variable
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Path to the saved map
    map_path = os.path.expanduser(os.path.join('~/mobile_robotics_ws', 'maps', 'map.yaml'))
    
    # Get the path to the launch file we want to include
    included_launch_file_dir1 = get_package_share_directory('turtlebot3_bringup')
    included_launch_file1 = os.path.join(included_launch_file_dir1, 'launch', 'turtlebot3_state_publisher.launch.py')
    included_launch_file_dir2 = get_package_share_directory('turtlebot3_navigation2')
    included_launch_file2 = os.path.join(included_launch_file_dir2, 'launch', 'navigation2.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Start the map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path}],
        output='screen'
    )

    # Start the state publisher node
    include_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file1),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Start the navigation node
    include_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file2),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_rviz': 'true'}.items(),
    )
    
    #Create a Node action for the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[
            {'use_sim_time': use_sim_time, 'use_rviz': 'false'},
        ],
        output='screen'
    )
    
    #lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        include_launch2,
        set_env,
        include_launch1,
        amcl_node,
        map_server_node, 
        lifecycle_manager_node,
        #you want to run the reach goal and then text_to_posestamp node manually
    ])
