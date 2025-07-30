import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_autonomous_robot = get_package_share_directory('autonomous_robot')
    world_path = os.path.join(pkg_autonomous_robot, 'worlds', 'small_city.world')


    return LaunchDescription([
        # start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
                        
            launch_arguments={
                'world': world_path,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # spawn TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            )
        ),

        # keyboard handle
        Node(
            package='keyboard',
            executable='keyboard',
            name='keyboard_node',
            output='screen',
        ),

        Node(
            package='autonomous_robot',
            executable='tb_key_control_node',
            output='screen',
            # prefix='xterm -e',  # for keyboard correct work
            name='tb_key_control_node'
        )
    ])