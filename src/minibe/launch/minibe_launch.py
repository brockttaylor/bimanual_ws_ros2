import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    declared_arguments = [] 
    launch_ik = LaunchConfiguration('ik')
    launch_ik_arg = DeclareLaunchArgument(
        'ik',
        default_value='true'
    )
    declared_arguments.append(launch_ik_arg)
    # state_publisher = LaunchConfiguration('state_publisher')
    state_publisher_launch_arg = DeclareLaunchArgument(
        'state_publisher',
        default_value='true'
    )
    declared_arguments.append(state_publisher_launch_arg)
    # rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )
    declared_arguments.append(rviz_arg)

    rd_file = DeclareLaunchArgument(
        'rd_file',
        default_value=os.path.join(get_package_share_directory('moveit_resources_panda_description'), "urdf", "panda.urdf")
    )
    declared_arguments.append(rd_file)
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="moveit_resources_panda_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    tf_prefix = LaunchConfiguration("tf_prefix")
    robot_file = os.path.join(get_package_share_directory('moveit_resources_panda_description'), "urdf", "panda.urdf")
    with open(robot_file, 'r') as infp:
        robot_description = infp.read()


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )
    launch_description = generate_move_group_launch(moveit_config)
    launch_description.add_entity(launch.LaunchDescription(declared_arguments + 
        [
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('state_publisher'))),
        #launch_ros.actions.Node(
        #    package='joint_state_publisher',
        #    executable='joint_state_publisher',
        #    name='joint_state_publisher'),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=["-d", rviz_config_file],),
        launch_ros.actions.Node(
            package='minibe',
            executable='ik',
            name='ik',
            parameters=[{"rd_file": LaunchConfiguration("rd_file")}],
            condition=IfCondition(LaunchConfiguration('ik')),
            output='screen'),]))
    return launch_description

