from launch import LaunchDescription
from launch_ros.actions import Node

# for event handlers
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete

# for pathjoin example
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

#for Launch Conditions
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, EnvironmentVariable

# Event handlers
'''def generate_launch_description():
    ld = LaunchDescription()
    turtlesim1 = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim1'
        )
    
    turtlesim2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim2'
    )
    # ld.add_action(turtlesim1_node)
    # return ld
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=turtlesim1,
                on_completion=[turtlesim2]
            )
        ),
        turtlesim1
    ])'''

#PathJoinSubstitution for constructing the path to a file
    # needs to have ur_description package installed
'''def generate_launch_description():
    
    rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file]
        ),
    ])'''

# Launch Conditions 
    # (tests if environment variable is true and changes output)
def generate_launch_description():
    return LaunchDescription([
        LogInfo(condition=IfCondition(
            PythonExpression(["'",
                EnvironmentVariable('USER'),
                "' == 'ubuntu'"
            ])
        ),
        msg='Welcome back James'
        )
    ])
