#include <visualNavigationInterface.hpp>


VisualNavigation::VisualNavigation(const std::string &configfile)
{
	YAML::Node fs = YAML::LoadFile(configfile);
	int sourceCount  = fs["source_count"].as<int>();
	std::string sourceConfig;
	
	VisionSystem* singleCam;
	for (unsigned int i = 0; i < sourceCount; i++)
	{
		sourceConfig =  fs["source_config_" + std::to_string(i+1)].as<std::string>();
		
		singleCam = new VisionSystem(sourceConfig);
		camVec.push_back(singleCam);
	}
	int useFiducial = fs["use_fiducial"].as<int>();
	if (useFiducial == 1)
	{
		std::string detectorConfig = fs["detector_config_path"].as<std::string>();
		std::string boardConfig = fs["board_config"].as<std::string>();
		int detectorType = fs["detector_type"].as<int>();
		int fiducialNavigationType = fs["fiducial_navigation_type"].as<int>();

		AnavigationSystem* navSys;
		if (fiducialNavigationType == 1)
		{
			navSys = new FiducialBoardNavigation(detectorType, camVec, detectorConfig, boardConfig);
			navSystems.push_back(navSys);
		}
		else if (fiducialNavigationType == 2)
		{
			navSys = new FiducialSlamNavigation(detectorType, camVec, detectorConfig, boardConfig);
			navSystems.push_back(navSys);
		}
		
		
	}
	// set time stamps for first system step
	currentSysTime = std::chrono::high_resolution_clock::now();
	lastSysTime = currentSysTime;

	// initUDPClient();
}


VisualNavigation::~VisualNavigation()
{

}

void VisualNavigation::estimatePosition()
{
	calcSystemLoopTime();
	for (unsigned i = 0; i < navSystems.size(); i++)
	{
		navSystems[i]->estimateState();
		navSystems[i]->setLoopTime(loopTime);
		
	}
	// position[0] = navSystems[0]->stateVector[0];
	// position[1] = navSystems[1]->stateVector[1];
	// position[2] = navSystems[2]->stateVector[2];
	// position[3] = navSystems[3]->stateVector[3];
	// position[4] = navSystems[4]->stateVector[4];
	// position[5] = navSystems[5]->stateVector[5];
	
	// sendMessageUDP(position);
}

void VisualNavigation::calcSystemLoopTime()
{
	currentSysTime = std::chrono::high_resolution_clock::now();
	timeStep = (currentSysTime - lastSysTime) / 1000;
	lastSysTime = currentSysTime;
	loopTime = timeStep.count();
}