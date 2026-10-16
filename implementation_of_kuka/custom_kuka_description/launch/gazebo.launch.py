import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF = "/home/rajesh/implementation_of_kuka/custom_kuka_description/urdf/custom_kuka_kr6.urdf"
WORLD = "/home/rajesh/implementation_of_kuka/custom_kuka_description/worlds/weld_cell.sdf"
PLAYER = "/home/rajesh/implementation_of_kuka/custom_kuka_description/scripts/weld_player.py"

def generate_launch_description():
    with open(URDF) as f:
        robot_desc = f.read()
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r " + WORLD}.items())
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_desc, "use_sim_time": True}])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "custom_kuka_kr6", "-z", "0.0"])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "--controller-manager", "/controller_manager"])
    player = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=["python3", PLAYER, "--mode", "trajectory"], output="screen",
        condition=IfCondition(LaunchConfiguration("play")))])
    return LaunchDescription([
        DeclareLaunchArgument("play", default_value="true"),
        gz, rsp, bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[player])),
    ])
