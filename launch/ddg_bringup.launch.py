from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    single_launch = "False"
    ddg_multi_robot_planner_package_share_directory = get_package_share_directory(
        "ddg_multi_robot_planner"
    )

    use_sim = LaunchConfiguration(
        "use_sim",
        default="False",
    )
    num_robots = LaunchConfiguration(
        "num_robots",
        default="2",
    )
    use_inbuilt_waypoint_follower = LaunchConfiguration(
        "use_inbuilt_waypoint_follower",
        default="False",
    )
    # Downsampled map parameters
    downsampled_map_file_path = LaunchConfiguration(
        "downsampled_map_file_path",
        default=os.path.join(
            ddg_multi_robot_planner_package_share_directory,
            "maps/downsampled-map/Oct-16/Oct-16-downsampled.map",
        ),
    )
    downsampling_factor = LaunchConfiguration(
        "downsampling_factor",
        default="30.0",
    )
    orignal_map_resolution = LaunchConfiguration(
        "orignal_map_resolution",
        default="0.03",
    )
    original_origin = LaunchConfiguration(
        "original_origin",
        default="[-19.7, -14.4]",
    )
    offset = LaunchConfiguration(
        "offset",
        default="[0.7, 0.95]",  # X axis Y axis
    )
    original_map_size = LaunchConfiguration(
        "original_map_size",
        default="[836, 1123]",
    )

    ddg_multi_robot_planner_node = Node(
        package="ddg_multi_robot_planner",
        executable="ddg_multi_robot_planner_node",
        output="screen",
        parameters=[
            {"use_sim": use_sim},
            {"num_robots": num_robots},
            {"use_inbuilt_waypoint_follower": use_inbuilt_waypoint_follower},
            {"downsampled_map_file_path": downsampled_map_file_path},
            {"downsampling_factor": downsampling_factor},
            {"orignal_map_resolution": orignal_map_resolution},
            {"original_origin": original_origin},
            {"offset": offset},
            {"original_map_size": original_map_size},
        ],
    )

    robot1_namespace = LaunchConfiguration("robot1_namespace", default="robot1")
    robot2_namespace = LaunchConfiguration("robot2_namespace", default="robot2")

    params = os.path.join(
        get_package_share_directory("robot_mission_control"), "config", "params.yaml"
    )

    mission_control_node = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(["not ", single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node",
                name="robot_mission_control_node",
                output="screen",
                parameters=[
                    {"robot1_namespace_param": robot1_namespace},
                    {"robot2_namespace_param": robot2_namespace},
                    {"mission_params": params},
                ],
            ),
            Node(
                condition=IfCondition(PythonExpression([single_launch])),
                package="robot_mission_control",
                executable="robot_mission_control_node_single",
                name="robot_mission_control_node_single",
                output="screen",
                parameters=[
                    {"robot_namespace_param": robot1_namespace},
                    {"mission_params": params},
                ],
            ),
        ]
    )

    # Add the node to the launch description
    ld = LaunchDescription()

    ld.add_action(ddg_multi_robot_planner_node)
    ld.add_action(mission_control_node)

    return ld
