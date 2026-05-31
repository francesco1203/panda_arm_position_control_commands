from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    clik_node = Node(
        package="pacchetto_nodi",
        executable="clik_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joint_traj_server = Node(
        package="pacchetto_nodi",
        executable="joint_traj_generator",
        output="screen"
    )

    cartesian_traj_server = Node(
        package="pacchetto_nodi",
        executable="cartesian_traj_generator",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        clik_node,
        joint_traj_server,
        cartesian_traj_server
        ])
