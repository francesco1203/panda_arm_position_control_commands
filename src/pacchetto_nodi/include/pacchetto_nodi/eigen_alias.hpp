#pragma once   // evita inclusioni doppie — alternativa moderna agli include guard

#include <Eigen/Dense>

//Eigen
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
//using Vector7d = Eigen::Matrix<double, 7, 1>;
using VectorXd = Eigen::VectorXd;                    //generic Vector dinamico
using MatrixXd = Eigen::MatrixXd;                    //generic Matrix dinamica
using Quaternion = Eigen::Quaterniond;
using RotoTraslMatrix = Eigen::Isometry3d;