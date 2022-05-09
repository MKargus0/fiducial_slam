#include "VisualNavigation.hpp"


VisualNavigation::VisualNavigation(const std::string &configfile)
{
	stateVector_.resize(6);
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
		camVec_.push_back(singleCam);
	}
	// Создаем обьект систему обнаружения реперных маркеров и навигационную систему
	if (useFiducial == 1)
	{
		std::string detectorConfig = fs["detector_config_path"].as<std::string>();
		std::string boardConfig = fs["board_config"].as<std::string>();
		int detectorType = fs["detector_type"].as<int>();
		int fiducialNavigationType = fs["fiducial_navigation_type"].as<int>();

		// создаем обьект навигационной системы
		ANavigationSystem* navSys;
		if (fiducialNavigationType == 1)
		{
			navSys = new FiducialBoardNavigation(detectorType, camVec_,
                                                 detectorConfig, boardConfig);
			navSystems_.push_back(navSys);
		}
		else if (fiducialNavigationType == 2)
		{
			navSys = new FiducialSlamNavigation(detectorType, camVec_,
                                                detectorConfig, boardConfig);
			navSystems_.push_back(navSys);
		}
		
		
	}
	// TODO add optical flow
	// устанавливаем начальные условия для рассчета времени цикла работы системы 
	currentSysTime_ = std::chrono::high_resolution_clock::now();
	lastSysTime_ = currentSysTime_;
}

VectorXd_t&	VisualNavigation::getStateVector()
{
	return (stateVector_);
}

void VisualNavigation::estimatePosition()
{	
	// рассчитываем время цикла работы системы
	calcSystemLoopTime();
	// получаем изображения со всех устройств
	for (auto & i : camVec_)
	{
		i->updateImage();
	}
	// оцениваем положение для всех навигационных систем
	for (auto & navSystem : navSystems_)
	{
		navStatus_ = navSystem->estimateState();
		navSystem->setLoopTime(loopTime_);
	}
	// TODO объединение решений для нескольких камер
	if(navStatus_ == 1)
	{
		stateVector_ = navSystems_[0]->getStateVector();
	}
	// sendMessageUDP(position);
}

void VisualNavigation::calcSystemLoopTime()
{
	// рассчет времени цикла работы системы
	currentSysTime_ = std::chrono::high_resolution_clock::now();
	timeStep_ = (currentSysTime_ - lastSysTime_) / 1000;
	lastSysTime_ = currentSysTime_;
	loopTime_ = timeStep_.count();
}

