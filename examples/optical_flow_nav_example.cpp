#include "opticalFlowNavigationSystem.hpp"


int main()
{
	std::string camConfig = "/home/argus/projects/visual_navigation/config/cameraConfigs/asusZenBookCamera.yaml";
	VisionSystem* singleCam;
	singleCam = new VisionSystem(camConfig);

	PhaseCorr flowNav(singleCam,60, 60, 
								640, 480);
	
	while(true)
	{
		singleCam->updateImage();
		flowNav.calculateMovement(0, 0, 0, 0);
		char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
	}
}