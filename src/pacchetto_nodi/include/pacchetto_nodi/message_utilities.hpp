#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard


#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"           //include anche Quaternion, che è un campo di Pose
#include "geometry_msgs/msg/pose_stamped.hpp" 


// Messaggi
using JointStateMsg  = sensor_msgs::msg::JointState;
using PoseMsg        = geometry_msgs::msg::Pose;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using QuaternionMsg  = geometry_msgs::msg::Quaternion;
