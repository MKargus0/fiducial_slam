#include <fuducialNavigationSystem.hpp>

#define ENABLE_QUIT_FROM_WINDOW

int main()
{
	std::string camConfig = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/ocamFIsheyeConfig2.yaml";
	std::string boardConfig = "/home/argus/projects/visual_navigation_new/config/fiducialBoards/board_1.yaml";
	std::string detectorConfig = "/home/argus/projects/visual_navigation_new/config/fiducialDetectorConfigs/dictionaryConfigARUCO.yaml";

	std::vector<VisionSystem*> camVec;
	VisionSystem* singleCam;
	singleCam = new VisionSystem(camConfig);
	camVec.push_back(singleCam);

	FiducialSlamNavigation navSys(0, camVec, detectorConfig, boardConfig);

	double step;
	std::chrono::_V2::system_clock::time_point current_time; // текущее время на шаге системы 
    std::chrono::_V2::system_clock::time_point last_time;    // время системы 
    std::chrono::duration<double, std::milli> time_step;	 // время цыкла управления системы, шаг разница между текшим и предыдушим шагом
	
	current_time = std::chrono::high_resolution_clock::now();
	last_time = current_time;
	while(true)
	{
		current_time = std::chrono::high_resolution_clock::now();
    	time_step = (current_time - last_time) / 1000;
		last_time = current_time;
		step = time_step.count();

		navSys.setDeltaTime(step);
		navSys.estimateState();
		char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
	}

}