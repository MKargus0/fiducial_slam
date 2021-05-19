#include <fiducialDetector.hpp>

#define ENABLE_QUIT_FROM_WINDOW

int main()
{
	std::string camConfigFile = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/asusZenBookCamera.yaml";
	std::string configFileDetector = "/home/argus/projects/visual_navigation_new/config/fiducialDetectorConfigs/dictionaryConfigARUCO.yaml";
	
	std::vector<VisionSystem*> camVec;
	VisionSystem* singleCam;
	singleCam = new VisionSystem(camConfigFile);
	camVec.push_back(singleCam);

	ArucoDetector arDet(camVec, configFileDetector);

	while(true)
	{

		arDet.detectFidusial();
		char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
	}
	
	return 0;
}