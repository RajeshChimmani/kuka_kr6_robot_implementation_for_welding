import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = "/home/rajesh/implementation_of_kuka/custom_kuka_description/urdf/custom_kuka_kr6.urdf"
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r default.sdf"}.items()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-world", "default", "-topic", "robot_description", "-name", "custom_kuka_kr6", "-z", "0.0"],
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        rsp,
        bridge,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[jsb_spawner]),
        TimerAction(period=9.0, actions=[arm_spawner])
    ])
