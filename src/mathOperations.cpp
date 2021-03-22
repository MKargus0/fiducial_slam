#include "markerMap.hpp"

/*
классический поворот вектора в кватернионной форме
*/
Eigen::Vector3d rotateVector(double X, double Y, double Z,
                    double roll, double pitch, double yaw)
{
    Eigen::Quaterniond q_vector(0, X, Y, Z); 
    Eigen::Quaterniond quat_yaw(std::cos(yaw/2),0,0,std::sin(yaw/2));
    Eigen::Quaterniond quat_pitch(std::cos(pitch/2),0,std::sin(pitch/2),0);
    Eigen::Quaterniond quat_roll(std::cos(roll/2),std::sin(roll/2),0,0);
    Eigen::Quaterniond final_rot = quat_yaw * quat_pitch * quat_roll;
    Eigen::Quaterniond res = final_rot * q_vector * final_rot.inverse();
    Eigen::Vector3d result = {res.x(),res.y(),res.z()};
    return result;
}
