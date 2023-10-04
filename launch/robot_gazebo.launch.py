import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
import xacro


def generate_launch_description():

    xacro_file_default = os.path.join(get_package_share_directory('gazebo_simulation_ros2'), 'description','manipulator_all.urdf.xacro')

    gazebo_ = LaunchConfiguration('gazebo_')

    gazebo_launch_arg = DeclareLaunchArgument(
        name='gazebo_',
        default_value= "true",
        description='Set to true if you want to simulate on gazebo, false otherwise.'
    )

    rviz_ = LaunchConfiguration("rviz_")

    rviz_launch_arg = DeclareLaunchArgument(
        name='rviz_',
        default_value= "true",
        description='Set to true if you want to visualize on rviz, false otherwise.'
    )

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file_default])

    rviz_absolute_path = os.path.join(get_package_share_directory('gazebo_simulation_ros2'),'rviz','visualize.rviz')


    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    node_joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
    )

    # Rviz2 node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_absolute_path],
        condition=IfCondition(rviz_)
    )

    # Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        condition=IfCondition(gazebo_),
        launch_arguments={'pause': 'true'}.items()
        #launch_arguments={'world': world_path}.items()
    )

    # entity spawn node (to spawn the robot from the /robot_description topic)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    # spawning the joint broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller","-c", "/controller_manager"],
        output="screen",
    )


    # Run the nodes
    return LaunchDescription([
        gazebo_launch_arg,
        rviz_launch_arg,
        node_robot_state_publisher,
        node_joint_state_publisher,
        launch_gazebo,
        spawn_entity,
        node_rviz,
        spawn_broadcaster,
        spawn_controller
    ])