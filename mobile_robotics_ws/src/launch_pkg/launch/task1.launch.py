from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration    
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get a list of all running nodes
    get_nodes = ExecuteProcess(
        cmd=['ros2', 'node', 'list'],
        output='capture',
        shell=True
    )

    # Run the command and get the output
    get_nodes.execute(None)
    nodes = get_nodes.output.split('\n')

    # Kill each node
    for node in nodes:
        if node:  # Skip empty lines
            kill_node = ExecuteProcess(
                cmd=['ros2', 'node', 'kill', node],
                shell=True
            )
            kill_node.execute(None)


# TASK1 
# setup the simulation of the TurtleBot3 burger robot using the
# Gazebo simulation environment and the TurtleBot3 Big House
# scenario. Note that during the exam evaluation, the robot can
# be spawned on the simulation scenario in a random position
# and additional obstacles can be present;


def generate_launch_description():
    # Set the environment variable
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Declare the launch arguments
    x_pose = DeclareLaunchArgument('x_pose', default_value='0.0', description='X position of the robot')
    y_pose = DeclareLaunchArgument('y_pose', default_value='0.5', description='Y position of the robot')

    # Get the path to the launch file we want to include
    included_launch_file_dir = get_package_share_directory('turtlebot3_gazebo')
    included_launch_file = os.path.join(included_launch_file_dir, 'launch', 'turtlebot3_big_house.launch.py')

    # Include the other launch file
    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file),
        launch_arguments={'x_pose': LaunchConfiguration('x_pose'),
                          'y_pose': LaunchConfiguration('y_pose')}.items(),
    )

    # Create a Node action THIS IS NOT WORKING BECAUSE THIS NODE REQUIRES AN AVAILABLE TERMINAL
    # my_node = Node(
    #     package='turtlebot3_teleop',  # Replace with the name of your package
    #     executable='teleop_keyboard',  # Replace with the name of your node
    #     name='teleop_keyboard',
    #     output='screen',
    #     #parameters=[...],  # Optional: replace with a list of your node's parameters
    # )
    # Create an ExecuteProcess action
    my_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
        output='screen',
    )

    return LaunchDescription([
        set_env,
        x_pose,
        y_pose,
        include_launch,
        my_node,
    ])