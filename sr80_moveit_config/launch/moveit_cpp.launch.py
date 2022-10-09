from ast import Param
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("SR80", package_name="sr80_moveit_config")
        .robot_description(file_path="config/SR80.urdf.xacro")
        .robot_description_semantic(file_path="config/SR80.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("sr80_moveit_config")
            + "/config/moveit_cpp.yaml"
            #file_path="/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("sr80_moveit_config", "config/sr80_simulated_config.yaml")
    # servo_params = {"moveit_servo": servo_yaml}

    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="/home/naci/dev_ws/src/sr80_moveit_config/config/sr80_simulated_config.yaml"
        )
        .to_dict()
    )

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="action_server",
        package="sr80_moveit_interface",
        executable="action_server",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.to_dict()],
    )

    moveitcpp_action_client_node = Node(
        name="action_client",
        package="sr80_moveit_interface",
        executable="action_client",
        output="screen",
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("sr80_moveit_config")
        + "/config/moveit_cpp.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            #"--controller-manager-timeout",
            #"300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("sr80_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    sr80_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sr80_arm_controller", "-c", "/controller_manager"],
    )

    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params,
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            rviz_node,
            ros2_control_node,
            sr80_arm_controller_spawner,
            moveit_cpp_node,
            moveitcpp_action_client_node,
            #servo_node,
        ]
    )