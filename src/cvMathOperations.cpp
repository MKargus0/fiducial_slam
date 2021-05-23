#include <cvMathOperations.hpp>

cv::Vec3d				CvMathOperations::rvecToEuler(cv::Vec3d &rvec)
{
	cv::Mat rodr;
	//переводим упрощеннный кватернион(rvec) в матрицу поворота
    cv::Rodrigues(rvec, rodr);
    //assert(isRotationMatrix(rodr));
    float sy = sqrt(rodr.at<double>(0,0) * rodr.at<double>(0,0) +  rodr.at<double>(1,0) * rodr.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
	//получаем углы из матрицы поворота
    if (!singular)
    {
        x = atan2(rodr.at<double>(2,1) , rodr.at<double>(2,2));
        y = atan2(-rodr.at<double>(2,0), sy);
        z = atan2(rodr.at<double>(1,0), rodr.at<double>(0,0));
    }
    else
    {
        x = atan2(-rodr.at<double>(1,2), rodr.at<double>(1,1));
        y = atan2(-rodr.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}

Eigen::Quaterniond  	CvMathOperations::eulerToQuat(cv::Vec3d &angles)
{
	// Eigen::Quaterniond quat_yaw(std::cos(angles[2]/2),0,0,std::sin(angles[2] / 2));
    // Eigen::Quaterniond quat_pitch(std::cos(angles[1]/2),0,std::sin(angles[1]/2),0);
    // Eigen::Quaterniond quat_roll(std::cos(angles[0]/2),std::sin(angles[0]/2),0,0);
    // Eigen::Quaterniond final_rot_q = quat_yaw * quat_pitch * quat_roll;
	//  return final_rot_q;
	Eigen::Quaterniond	Q;
	Q.w() = std::cos(angles[2] / 2) * std::cos(angles[1] / 2) * std::cos(angles[0] / 2) -
		    std::sin(angles[2] / 2) * std::sin(angles[1] / 2) * std::sin(angles[0] / 2);
	Q.x() = std::cos(angles[2] / 2) * std::cos(angles[1] / 2) * std::sin(angles[0] / 2) +
		    std::sin(angles[2] / 2) * std::sin(angles[1] / 2) * std::cos(angles[0] / 2);
	Q.y() = std::sin(angles[2] / 2) * std::cos(angles[1] / 2) * std::cos(angles[0] / 2) +
		    std::cos(angles[2] / 2) * std::sin(angles[1] / 2) * std::sin(angles[0] / 2);
	Q.z() = std::cos(angles[2] / 2) * std::sin(angles[1] / 2) * std::cos(angles[0] / 2) -
		    std::sin(angles[2] / 2) * std::cos(angles[1] / 2) * std::sin(angles[0] / 2);
	return Q;
}
   

cv::Vec3d           	CvMathOperations::visionPoseTransformToLocalFrame(cv::Vec3d &tvec, cv::Vec3d &rvec)
{
	cv::Mat result;
    cv::Mat rodr;
    cv::Rodrigues(rvec, rodr);
    result = -rodr.t() *  cv::Mat(tvec);
    return cv::Vec3d(result);
}

cv::Point3f			CvMathOperations::rotateCorn(cv::Point3f corn, double boardX, double boardY, double boardZ, double roll, double pitch, double yaw)
{
	Eigen::Quaternionf q_vector(0, corn.x - boardX, corn.y - boardY, corn.z - boardZ); 
    Eigen::Quaternionf quat_yaw(std::cos(yaw/2) , 0, 0, std::sin(yaw/2));
    Eigen::Quaternionf quat_pitch(std::cos(pitch/2), 0, std::sin(pitch/2), 0);
    Eigen::Quaternionf quat_roll(std::cos(roll/2), std::sin(roll/2), 0, 0);

    Eigen::Quaternionf final_rot_q = quat_yaw * quat_pitch * quat_roll;
    // cout <<std::to_string(final_rot_q.w()) <<endl;
    // cout <<std::to_string(final_rot_q.x()) <<endl;
    // cout <<std::to_string(final_rot_q.y()) <<endl;
    // cout <<std::to_string(final_rot_q.z()) <<endl;
    //Eigen::Quaterniond qua_pitch(1,1,1,1);
    Eigen::Quaternionf res = final_rot_q * q_vector * final_rot_q.inverse();

    cv::Point3d result = {res.x() + boardX, res.y() + boardY, res.z() + boardZ};
    return result;

}