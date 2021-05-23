#ifndef MATH_OPERATIONS_HPP
#define MATH_OPERATIONS_HPP
#include <Eigen/Geometry>
#include <types.hpp>

Eigen::Vector3d rotateVector(Eigen::Vector3d &position, Eigen::Vector3d &orientation);
Eigen::Vector3d rotateVector(VectorXd &position);


#endif