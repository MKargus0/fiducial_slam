#include <visualNavigationInterface.hpp>

// #define ENABLE_QUIT_FROM_WINDOW

int main()
{
	std::string config = "/home/argus/projects/visual_navigation/config/navigation_system_config.yaml";

	VisualNavigation visNavInterface(config);

	while(true)
	{
		double step;
		std::chrono::_V2::system_clock::time_point currentSysTime; // текущее время на шаге системы 
    	std::chrono::_V2::system_clock::time_point lastSysTime;    // время системы 
    	std::chrono::duration<double, std::milli> timeStep;	 // время цикла управления системы, шаг разница между текшим и предыдушим шагом
		currentSysTime = std::chrono::high_resolution_clock::now();
		lastSysTime = currentSysTime;

		for (unsigned int i = 0; i < visNavInterface.camVec.size(); i++)
		{
			visNavInterface.camVec[i]->updateImage();
		}

		visNavInterface.estimatePosition();
		char key = (char) cv::waitKey(1);
   			if (key == 27)
        		break;
	}
}