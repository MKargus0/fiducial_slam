#ifndef COMMON_TOOLS_MATH_OPERATIONS_HPP
#define COMMON_TOOLS_MATH_OPERATIONS_HPP

#include "types.hpp"

#include <Eigen/Geometry>

Eigen::Vector3d rotateVector(Eigen::Vector3d &position, Eigen::Vector3d &orientation);
Eigen::Vector3d rotateVector(VectorXd_t &position);


#endif