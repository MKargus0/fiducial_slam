#pragma once
#include <types.hpp>

//мптематические преобразования над типами данных cv::
class CvMathOperations
{
	public:
		static cv::Vec3d			rvecToEuler(cv::Vec3d &rvec);
		static Eigen::Quaterniond  	eulerToQuat(cv::Vec3d &angles);
		static cv::Vec3d           	visionPoseTransformToLocalFrame(cv::Vec3d &tvec, cv::Vec3d &rvec);
};