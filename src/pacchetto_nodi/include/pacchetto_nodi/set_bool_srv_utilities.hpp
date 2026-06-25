#pragma once

//#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>

//alias
using SetBoolSrv = std_srvs::srv::SetBool; 
using SetBoolRequestPtr = std::shared_ptr<SetBoolSrv::Request>;
using SetBoolResponsePtr = std::shared_ptr<SetBoolSrv::Response>;

//server
using SetBoolServerPtr = rclcpp::Service<SetBoolSrv>::SharedPtr;

//client
using SetBoolClientPtr = rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr;

