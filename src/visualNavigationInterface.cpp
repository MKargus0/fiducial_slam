#include <visualNavigationInterface.hpp>


VisualNavigation::VisualNavigation(const std::string &configfile)
{
	// читаем файл конфигурации
	YAML::Node fs = YAML::LoadFile(configfile);
	int sourceCount  = fs["source_count"].as<int>();
	int useFiducial = fs["use_fiducial"].as<int>();
	int useOpticalFlow = fs["use_optical_flow"].as<int>();
	std::string sourceConfig;
	// инициализируем источники изображения
	VisionSystem* singleCam;
	for (unsigned int i = 0; i < sourceCount; i++)
	{
		sourceConfig =  fs["source_config_" + std::to_string(i+1)].as<std::string>();
		singleCam = new VisionSystem(sourceConfig);
		camVec.push_back(singleCam);
	}
	// Создаем обьект систему обнаружения реперных маркеров и навигационную систему
	if (useFiducial == 1)
	{
		std::string detectorConfig = fs["detector_config_path"].as<std::string>();
		std::string boardConfig = fs["board_config"].as<std::string>();
		int detectorType = fs["detector_type"].as<int>();
		int fiducialNavigationType = fs["fiducial_navigation_type"].as<int>();

		// создаем обьект навигационной системы
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
	if (useOpticalFlow == 1)
	{
		int flowType = fs["flow_type"].as<int>();
		double fieldOfViewAngleWidth = fs["fieldOfViewAngleWidth"].as<int>();
		double fieldOfViewAngleHeight = fs["fieldOfViewAngleHeight"].as<int>();
		int fieldWidth = fs["fieldWidth"].as<int>();
		int fieldHeight = fs["fieldHeight"].as<int>();
		if (flowType == 1)
		{
			flowNav = new PhaseCorr(singleCam,fieldOfViewAngleWidth, fieldOfViewAngleHeight, 
								fieldWidth, fieldHeight);
		}

	}
	// устанавливаем начальные условия для рассчета времени цикла работы системы 
	currentSysTime = std::chrono::high_resolution_clock::now();
	lastSysTime = currentSysTime;

	// initUDPClient();
}


VisualNavigation::~VisualNavigation()
{

}

void VisualNavigation::estimatePosition()
{	
	// рассчитываем время цикла работы системы
	calcSystemLoopTime();
	// оцениваем положение для всех навигационных систем
	for (unsigned i = 0; i < navSystems.size(); i++)
	{
		nav_status = navSystems[i]->estimateState();
		navSystems[i]->setLoopTime(loopTime);
	}
	// TODO объединение решений для нескольких камер
	if(nav_status == 1)
	{
		position[0] = navSystems[0]->stateVector[0];
		position[1] = navSystems[0]->stateVector[1];
		position[2] = navSystems[0]->stateVector[2];
		position[3] = navSystems[0]->stateVector[3];
		position[4] = navSystems[0]->stateVector[4];
		position[5] = navSystems[0]->stateVector[5];
	}
	
	// sendMessageUDP(position);
}

void VisualNavigation::calcSystemLoopTime()
{
	// рассчет времени цыкла работы системы
	currentSysTime = std::chrono::high_resolution_clock::now();
	timeStep = (currentSysTime - lastSysTime) / 1000;
	lastSysTime = currentSysTime;
	loopTime = timeStep.count();
}

