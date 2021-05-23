#include <fiducialDetector.hpp>

#define ENABLE_QUIT_FROM_WINDOW

int main()
{
	std::string camConfigFile1 = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/asusZenBookCamera.yaml";
	std::string	camConfigFile2 = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/ocamFIsheyeConfig.yaml";
	std::string	camConfigFile3 = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/ocamFIsheyeConfig2.yaml";
	std::string configFileDetector = "/home/argus/projects/visual_navigation_new/config/fiducialDetectorConfigs/dictionaryConfigARUCO.yaml";
	
	std::vector<VisionSystem*> camVec;
	VisionSystem* singleCam;
	singleCam = new VisionSystem(camConfigFile1);
	camVec.push_back(singleCam);
	singleCam = new VisionSystem(camConfigFile2);
	camVec.push_back(singleCam);
	singleCam = new VisionSystem(camConfigFile3);
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