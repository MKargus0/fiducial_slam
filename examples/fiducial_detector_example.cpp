#include "FiducialDetector.hpp"

#define ENABLE_QUIT_FROM_WINDOW

int main()
{
  std::string camConfigFile1 = "../config/cameraConfigs/asusZenBookCamera.yaml";
  std::string configFileDetector = "../config/fiducialDetectorConfigs/dictionaryConfigARUCO.yaml";

  //use multiple camera for detecting fiducials
  std::vector<VisionSystem*> camVec;
  VisionSystem* singleCam;
  singleCam = new VisionSystem(camConfigFile1);
  camVec.push_back(singleCam);
//  singleCam = new VisionSystem(camConfigFile2);
//  camVec.push_back(singleCam);
//  singleCam = new VisionSystem(camConfigFile3);
//  camVec.push_back(singleCam);

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