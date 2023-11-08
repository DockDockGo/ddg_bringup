from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction


def generate_launch_description():
    single_launch = "True"

    ddg_multi_robot_planner_node = Node(
        package="ddg_multi_robot_planner",
        executable="ddg_multi_robot_planner_node",
        output="screen",
    )

    mission_control_node = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(["not ", single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node",
                name="robot_mission_control_node",
                output="screen",
            ),
            Node(
                condition=IfCondition(PythonExpression([single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node_single",
                name="robot_mission_control_node_single",
                output="screen",
            ),
        ]
    )

    # Add the node to the launch description
    ld = LaunchDescription()

    ld.add_action(ddg_multi_robot_planner_node)
    ld.add_action(mission_control_node)

    return ld
