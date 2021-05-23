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

	FiducialBoardNavigation navSys(0, camVec, detectorConfig, boardConfig);

	while(true)
	{

		navSys.estimateState();
		char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
	}
}