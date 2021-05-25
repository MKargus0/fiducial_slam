#include <mathOperations.hpp>


int main()
{
	// x
	// y
	// z
	// pitch
	// roll
	// yaw
	VectorXd vecNotRot(6);
	// vecNotRot[0] = -2.02974; 
	// vecNotRot[1] = -3.26611;
	// vecNotRot[2] =  1.8547;
	// vecNotRot[3] = -1.57 / 2;
	// vecNotRot[4] = 0;
	// vecNotRot[5] = 0;


	vecNotRot[0] = 2.07614;
	vecNotRot[1] = -1.92947; 
	vecNotRot[2] =  2.83263;
	vecNotRot[3] = -0.830594;
	vecNotRot[4] = 0;
	vecNotRot[5] = -3.12346;

	// vecNotRot[0] = 0;
	// vecNotRot[1] = 1; 
	// vecNotRot[2] = 0;
	// vecNotRot[3] = 0; 
	// vecNotRot[4] = 0;
	// vecNotRot[5] = 0;
	Eigen::Vector3d vecRot;

	vecRot = rotateVector(vecNotRot);

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