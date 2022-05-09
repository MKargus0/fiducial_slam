#include "FiducialNavigationSystem.hpp"

#define ENABLE_QUIT_FROM_WINDOW

int main()
{
	std::string camConfig = "../config/cameraConfigs/ocamFIsheyeConfig2.yaml";
	std::string boardConfig = "..config/fiducialBoards/board_2.yaml";
	std::string detectorConfig = "../config/fiducialDetectorConfigs/dictionaryConfigARUCO.yaml";

	std::vector<VisionSystem*> camVec;
	VisionSystem* singleCam;
	singleCam = new VisionSystem(camConfig);
	camVec.push_back(singleCam);

	FiducialSlamNavigation navSys(0, camVec, detectorConfig, boardConfig);

	double step;
	std::chrono::_V2::system_clock::time_point currentSysTime; // текущее время на шаге системы 
    std::chrono::_V2::system_clock::time_point lastSysTime;    // время системы 
    std::chrono::duration<double, std::milli> timeStep;	 // время цикла управления системы, шаг разница между текшим и предыдушим шагом
	
	currentSysTime = std::chrono::high_resolution_clock::now();
	lastSysTime = currentSysTime;
	// главный цыкл алгоритма
	while(true)
	{
		currentSysTime = std::chrono::high_resolution_clock::now();
    	timeStep = (currentSysTime - lastSysTime) / 1000;
		lastSysTime = currentSysTime;
		step = timeStep.count();

		for (unsigned int i = 0; i < camVec.size(); i++)
		{
			camVec[i]->updateImage();
		}

		navSys.setLoopTime(step);
		navSys.estimateState();
		char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
	}

}