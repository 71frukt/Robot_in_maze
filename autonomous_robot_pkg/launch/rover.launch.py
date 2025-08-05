import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_gazebo_ros        = get_package_share_directory('gazebo_ros')
    autonomous_robot_pkg  = get_package_share_directory('autonomous_robot_pkg')
    rover_model_xacro     = os.path.join(autonomous_robot_pkg, 'models', 'rover', 'rover_model.xacro')
    rover_controller_yaml = os.path.join(autonomous_robot_pkg, 'models', 'rover', 'rover_diff_drive_controller.yaml')

    robot_description_content = ParameterValue(
        Command(['xacro', ' ', rover_model_xacro]),
        value_type=str
    )

    # robot_state_publisher — публикует robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # ros2_control_node — НЕ получает robot_description напрямую
    start_controller = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[rover_controller_yaml],  # только YAML!
        output='screen'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover', 
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Загрузка и активация контроллеров через spawner
    start_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )


    return LaunchDescription([
        robot_state_publisher,
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot,
        start_controller,
        start_joint_broadcaster,
        start_diff_drive
    ])