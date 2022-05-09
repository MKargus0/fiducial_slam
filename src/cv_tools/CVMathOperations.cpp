#include "CVMathOperations.hpp"

cv::Vec3d				CvMathOperations::rvecToEuler(cv::Vec3d &rvec)
{
    cv::Vec3d   result;
	cv::Mat     rodr;
	// переводим вектор вращения(rvec) в матрицу поворота
    cv::Rodrigues(rvec, rodr);
    float sy = sqrt(rodr.at<double>(0,0) * rodr.at<double>(0,0) +  rodr.at<double>(1,0) * rodr.at<double>(1,0) );
    bool singular = sy < 1e-6;
	//получаем углы из матрицы поворота
    if (!singular)
    {
        result[0] = atan2(rodr.at<double>(2,1) , rodr.at<double>(2,2));
        result[1] = atan2(-rodr.at<double>(2,0), sy);
        result[2] = atan2(rodr.at<double>(1,0), rodr.at<double>(0,0));
    }
    else
    {
        result[0] = atan2(-rodr.at<double>(1,2), rodr.at<double>(1,1));
        result[1] = atan2(-rodr.at<double>(2,0), sy);
        result[2] = 0;
    }
    return (result);
}


Eigen::Vector3d		CvMathOperations::getAngleDifferense(cv::Vec3d &rvecMap, cv::Vec3d &rvecMarker)
{
    Eigen::Vector3d result;
	cv::Mat rodrMap;
	cv::Mat rodrMarker;
	cv::Mat rodrRes;

	//получаем матрицы поворота из вектора вращения для СК карты и СК маркера
    cv::Rodrigues(rvecMap, rodrMap);
	cv::Rodrigues(rvecMarker, rodrMarker);

	// выполняем переход из СК маркера в СК камеры и затем в СК карты
	// на выходе получаем матрицу поворота между СК карты и маркера
	rodrRes = rodrMap.t() * rodrMarker;

	float sy = sqrt(rodrRes.at<double>(0,0) * rodrRes.at<double>(0,0) +  rodrRes.at<double>(1,0) * rodrRes.at<double>(1,0) );
    bool singular = sy < 1e-6;
	// получаем углы из матрицы поворота
    if (!singular)
    {
        result[0] = atan2(rodrRes.at<double>(2,1) , rodrRes.at<double>(2,2));
        result[1] = atan2(-rodrRes.at<double>(2,0), sy);
        result[2] = atan2(rodrRes.at<double>(1,0), rodrRes.at<double>(0,0));
    }
    else
    {
        result[0] = atan2(-rodrRes.at<double>(1,2), rodrRes.at<double>(1,1));
        result[1] = atan2(-rodrRes.at<double>(2,0), sy);
        result[2] = 0;
    }

    return (result);
}


Eigen::Quaterniond  	CvMathOperations::eulerToQuat(cv::Vec3d &angles)
{
	Eigen::Quaterniond	Q;
	Q.w() = std::cos(angles[2] / 2) * std::cos(angles[1] / 2) * std::cos(angles[0] / 2) -
		    std::sin(angles[2] / 2) * std::sin(angles[1] / 2) * std::sin(angles[0] / 2);
	Q.x() = std::cos(angles[2] / 2) * std::cos(angles[1] / 2) * std::sin(angles[0] / 2) +
		    std::sin(angles[2] / 2) * std::sin(angles[1] / 2) * std::cos(angles[0] / 2);
	Q.y() = std::sin(angles[2] / 2) * std::cos(angles[1] / 2) * std::cos(angles[0] / 2) +
		    std::cos(angles[2] / 2) * std::sin(angles[1] / 2) * std::sin(angles[0] / 2);
	Q.z() = std::cos(angles[2] / 2) * std::sin(angles[1] / 2) * std::cos(angles[0] / 2) -
		    std::sin(angles[2] / 2) * std::cos(angles[1] / 2) * std::sin(angles[0] / 2);
	return (Q);
}
   
cv::Vec3d           	CvMathOperations::visionPoseTransformToLocalFrame(cv::Vec3d &tvec, cv::Vec3d &rvec)
{
	cv::Mat result;
    cv::Mat rodr;
    cv::Rodrigues(rvec, rodr);
    result = -rodr.t() *  cv::Mat(tvec);
    return cv::Vec3d(result);
}

cv::Point3f			CvMathOperations::rotateCorn(cv::Point3f corn, double boardX, double boardY,
                                                 double boardZ, double roll, double pitch, double yaw)
{
	cv::Point3d result;
	// записываем вектор в виде кватерниона для осуществления операции умножения
	Eigen::Quaterniond rotVector(0, corn.x - boardX, corn.y - boardY, corn.z - boardZ); 
    Eigen::Quaterniond qYaw(std::cos(yaw / 2) , 0, 0, std::sin(yaw / 2));
    Eigen::Quaterniond qPitch(std::cos(pitch / 2), 0, std::sin(pitch / 2), 0);
    Eigen::Quaterniond qRoll(std::cos(roll / 2), std::sin(roll / 2), 0, 0);
	// Получаем кватернион ориентации
    Eigen::Quaterniond finalRot = qYaw * qPitch * qRoll;

	// поворачиваем вектор
    Eigen::Quaterniond res = finalRot * rotVector * finalRot.inverse();

	result.x = res.x() + boardX;
	result.y = res.y() + boardY;
	result.z = res.z() + boardZ;
    return (result);
}