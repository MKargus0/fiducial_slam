#include <mathOperations.hpp>

Eigen::Vector3d rotateVector(Eigen::Vector3d &position, Eigen::Vector3d &orientation)
{
	Eigen::Quaterniond	Q;
	Eigen::Quaterniond	Pose;
	Eigen::Vector3d 	result;
	Pose.w() = 0;
	Pose.x() = position[0];
	Pose.y() = position[1];
	Pose.z() = position[2];
	Q.w() = std::cos(orientation[2] / 2) * std::cos(orientation[1] / 2) * std::cos(orientation[0] / 2) -
		    std::sin(orientation[2] / 2) * std::sin(orientation[1] / 2) * std::sin(orientation[0] / 2);
	Q.x() = std::cos(orientation[2] / 2) * std::cos(orientation[1] / 2) * std::sin(orientation[0] / 2) +
		    std::sin(orientation[2] / 2) * std::sin(orientation[1] / 2) * std::cos(orientation[0] / 2);
	Q.y() = std::sin(orientation[2] / 2) * std::cos(orientation[1] / 2) * std::cos(orientation[0] / 2) +
		    std::cos(orientation[2] / 2) * std::sin(orientation[1] / 2) * std::sin(orientation[0] / 2);
	Q.z() = std::cos(orientation[2] / 2) * std::sin(orientation[1] / 2) * std::cos(orientation[0] / 2) -
		    std::sin(orientation[2] / 2) * std::cos(orientation[1] / 2) * std::sin(orientation[0] / 2);
	Pose = Q * Pose * Q.inverse();
	result[0] = Pose.x();
	result[1] = Pose.y();
	result[2] = Pose.z();
	return result;
}



Eigen::Vector3d rotateVector(VectorXd &position)
{
	Eigen::Quaterniond	Q;
	Eigen::Quaterniond	Pose;
	Eigen::Vector3d 	result;
	Pose.w() = 0;
	Pose.x() = position[0];
	Pose.y() = position[1];
	Pose.z() = position[2];
	Q.w() = std::cos(position[5] / 2) * std::cos(position[4] / 2) * std::cos(position[3] / 2) -
		    std::sin(position[5] / 2) * std::sin(position[4] / 2) * std::sin(position[3] / 2);
	Q.x() = std::cos(position[5] / 2) * std::cos(position[4] / 2) * std::sin(position[3] / 2) +
		    std::sin(position[5] / 2) * std::sin(position[4] / 2) * std::cos(position[3] / 2);
	Q.y() = std::sin(position[5] / 2) * std::cos(position[4] / 2) * std::cos(position[3] / 2) +
		    std::cos(position[5] / 2) * std::sin(position[4] / 2) * std::sin(position[3] / 2);
	Q.z() = std::cos(position[5] / 2) * std::sin(position[4] / 2) * std::cos(position[3] / 2) -
		    std::sin(position[5] / 2) * std::cos(position[4] / 2) * std::sin(position[3] / 2);
	Pose = Q * Pose * Q.inverse();
	result[0] = Pose.x();
	result[1] = Pose.y();
	result[2] = Pose.z();
	return result;
}