from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF = "/home/rajesh/implementation_of_kuka/custom_kuka_description/urdf/custom_kuka_kr6.urdf"
RVIZ = "/home/rajesh/implementation_of_kuka/custom_kuka_description/rviz/kuka_weld.rviz"
PLAYER = "/home/rajesh/implementation_of_kuka/custom_kuka_description/scripts/weld_player.py"

def generate_launch_description():
    with open(URDF) as f:
        robot_desc = f.read()
    replay = LaunchConfiguration("replay")
    return LaunchDescription([
        DeclareLaunchArgument("replay", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_desc}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", output="screen",
             condition=UnlessCondition(replay)),
        ExecuteProcess(cmd=["python3", PLAYER, "--mode", "marker"], output="screen", condition=UnlessCondition(replay)),
        ExecuteProcess(cmd=["python3", PLAYER, "--mode", "jointstate", "--loop"], output="screen", condition=IfCondition(replay)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", RVIZ]),
    ])
