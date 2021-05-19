#include <visualSystem.hpp>
#include <visualizationOnImage.hpp>

#define ENABLE_QUIT_FROM_WINDOW
int main()
{
	std::string configFile = "/home/argus/projects/visual_navigation_new/config/cameraConfigs/asusZenBookCamera.yaml";
	VisionSystem singleCam(configFile);
	while(true)
	{
		singleCam.inputVideo >> singleCam.image;
		if(!singleCam.image.empty())
		{
			VisualizationOnImage::showImage(singleCam.image);
			//exit if pres esc
			char key = (char) cv::waitKey(30);
   			if (key == 27)
        		break;
		}

	}
	
}