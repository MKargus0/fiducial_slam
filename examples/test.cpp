#include <mathOperations.hpp>


// std::vector<double>		getWallsAngles(Eigen::Vector3d vector2, Eigen::Vector3d vector2)
// {
// 	std::vector<double> result;

// 	double angleYaw = 0.0;
// 	double anglePitch = 0.0;
// 	double angleRoll = 0.0;
// 	//base vector
// 	Eigen::Vector3d v1;
// 	Eigen::Vector3d v2;
// 	Eigen::Vector3d p;

// 	v2 = {0,centralNext[1], centralNext[2]};
// 	p = {0, centralPast[1],  centralPast[2]};
// 	v2 = v2 - p;

// 	angleRoll =  std::atan2(v2[0],v2[2]);
// 	result.push_back(angleRoll);

// 	v1 = {1,0,0};
// 	v2 = {centralNext[0], 0, centralNext[2]};
// 	p = {centralPast[0], 0, centralPast[2]};
// 	v2 = v2 - p;
// 	anglePitch = std::atan2(v2[2],v2[1]);;

// 	result.push_back(anglePitch);

// 	v1 = {1,0,0};
// 	v2 = {centralNext[0], centralNext[1], 0};
// 	p = {centralPast[0], centralPast[1], 0};
// 	v2 = v2 - p;
	
// 	angleYaw = std::atan2(v2[1],v2[0]);

// 	result.push_back(angleYaw);
// 	std::cout << "roll" << angleRoll << std::endl;
// 	std::cout << "pitch" << anglePitch << std::endl;
// 	std::cout << "yaw" << angleYaw << std::endl;

// 	return result;
//   // return angle;
  
// }



int main()
{
	// x
	// y
	// z
	// pitch
	// roll
	// yaw
	VectorXd vecNotRot(6);
	// VectorXd vecMap(6);
	Eigen::Vector3d vecMap;
	// vecNotRot[0] = -2.02974; 
	// vecNotRot[1] = -3.26611;
	// vecNotRot[2] =  1.8547;
	// vecNotRot[3] = -1.57 / 2;
	// vecNotRot[4] = 0;
	// vecNotRot[5] = 0;


	vecNotRot[0] =  -1.07844;
	vecNotRot[1] =   -0.027983; 
	vecNotRot[2] =   2.73837;
	vecNotRot[3] = 	0;
	vecNotRot[4] =  0;
	vecNotRot[5] = -1.56845;

	// vecNotRot[0] = 0;
	// vecNotRot[1] = 1; 
	// vecNotRot[2] = 0;
	// vecNotRot[3] = 0; 
	// vecNotRot[4] = 0;
	// vecNotRot[5] = 0;




	vecMap[0] = -0.0970007;
	vecMap[1] = -0.592156; 
	vecMap[2] = 2.7601;
	
	Eigen::Vector3d vecRot;

	vecRot = rotateVector(vecNotRot);
	// vecRot[0] = vecNotRot[0];
	// vecRot[1] = vecNotRot[1];
	// vecRot[2] = vecNotRot[2];

	std::cout << vecRot << std::endl << " " << std::endl;

	vecRot[0] = vecMap[0] - vecRot[0];
	vecRot[1] = vecMap[1] - vecRot[1];
	vecRot[2] = vecMap[2] - vecRot[2];
	std::cout << vecRot << std::endl;

	// Eigen::Matrix3f m;

	// m = Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitX())
	//   * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitY())
	//   * Eigen::AngleAxisf(0.33*M_PI, Eigen::Vector3f::UnitZ());
  
	// std::cout << "original rotation:" << std::endl;
	// std::cout << m << std::endl << std::endl;

	// Eigen::Vector3f ea = m.eulerAngles(0, 1, 2); 
	// std::cout << "to Euler angles:" << std::endl;
	// std::cout << ea << std::endl << std::endl;

	// Eigen::Matrix3f n;
	// n = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitX())
	//   * Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY())
	//   * Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitZ()); 
  
	// std::cout << "recalc original rotation:" << std::endl;
	// std::cout << n << std::endl;

}