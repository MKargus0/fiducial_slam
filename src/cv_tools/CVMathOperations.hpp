#ifndef CV_TOOLS_CV_MATH_OPERATIONS
#define CV_TOOLS_CV_MATH_OPERATIONS

#include "types.hpp"

//мптематические преобразования над типами данных cv::
class CvMathOperations
{
	public:
		static	cv::Vec3d				rvecToEuler(cv::Vec3d &rvec);
		static	Eigen::Quaterniond  	eulerToQuat(cv::Vec3d &angles);
		static	cv::Vec3d           	visionPoseTransformToLocalFrame(cv::Vec3d &tvec, cv::Vec3d &rvec);
		static	cv::Point3f				rotateCorn(cv::Point3f corn, double boardX, double boardY, double boardZ,
													double roll, double pitch, double yaw);
		static	Eigen::Vector3d			getAngleDifferense(cv::Vec3d &rvecMap, cv::Vec3d &rvecMarker);
};

#endif