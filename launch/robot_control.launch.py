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


    return LaunchDescription([
        # start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            # launch_arguments={'gui_plugin': ''}.items()
        ),

        # spawn TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            )
        ),

        # Node(
        #     package='autonomous_tb3',
        #     executable='tb_key_control_node',
        #     output='screen',
        #     # prefix='xterm -e',  # for keyboard correct work
        #     name='tb_key_controller_node'
        # )
    ])