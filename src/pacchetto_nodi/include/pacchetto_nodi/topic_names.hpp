#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <string>

//from simulator
const std::string READING_JOINT_STATES_TOPIC = "/joint_states";
const std::string PUBLISH_JOINT_COMMAND_TOPIC = "/cmd/joint_position";

//created
const std::string CARTESIAN_DESIRED_POSE_TOPIC = "desired_cartesian_pose";

