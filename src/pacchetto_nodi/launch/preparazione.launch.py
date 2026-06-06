from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    joint_traj_server = Node(
        package="pacchetto_nodi",
        executable="joint_traj_generator",
        output="screen",
        parameters=[{
            'T_camp': 0.1,      # Periodo di campionamento traiettoria generata (10Hz)
            'qdd_c': 1.5        # Accelerazione del profilo trapezoidale
        }]    
    )

    cartesian_traj_server = Node(
        package="pacchetto_nodi",
        executable="cartesian_traj_generator",
        output="screen",
        parameters=[
            # Configurazione di MoveIt
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,

            # Iperparametri algoritmici
            {
                "T_camp": 0.1,          # Campionamento generazione traiettoria a 10Hz
                "cdd_c_trasl": 1.0,     # Accelerazione traslazionale personalizzata (m/s^2) in profilo trapezoidale
                "cdd_c_rot": 1.0        # Accelerazione rotazionale personalizzata (rad/s^2) in profilo trapezoidale
            }
        ],
    )

    clik_node = Node(
        package="pacchetto_nodi",
        executable="clik_node",
        output="screen",
        parameters=[
            # Configurazione di MoveIt
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,

            # Iperparametri algoritmici
            {
                "Tclik": 0.001,                       # Frequenza di calcolo del clik
                "gamma_on_T": 0.5,                    # guadagno del clik
                "singularity_trshld_warn": 0.01,      # soglia di detection -> vicini alla singolarità
                "singularity_trshld_error": 0.001     # soglia di errore -> siamo in singolarità
            }
        ],
    )

    return LaunchDescription([
        joint_traj_server,
        cartesian_traj_server,
        clik_node
    ])
