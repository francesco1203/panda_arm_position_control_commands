//descrizione file: elementi della rete ROS

#pragma once
#include <string>


/*TOPIC*/

//from simulator
const std::string READING_JOINT_STATES_TOPIC = "/joint_states";
const std::string PUBLISH_JOINT_COMMAND_TOPIC = "/cmd/joint_position";

//created
const std::string CARTESIAN_DESIRED_POSE_TOPIC = "desired_cartesian_pose";


/*ACTIONS*/
const std::string MOVE_CARTESIAN_ACTION = "cartesian_traj_action";
const std::string MOVE_JOINT_ACTION = "move_joint_lin_action";

/*SERVICES*/
const std::string CLIK_SERVICE_ON_OFF = "clik_on_off";

