#include <fiducialDetector.hpp>

AfiducialDetector::AfiducialDetector(std::vector<VisionSystem*>	&visionSysVector)
{
	this->visionSysVector = visionSysVector;
}

AfiducialDetector::~AfiducialDetector()
{
	//очищаем память
	for (unsigned int i = 0; i < visionSysVector.size(); i++)
		delete visionSysVector[i];
	
}

ArucoDetector::ArucoDetector(std::vector<VisionSystem*>	&visionSysVector, const std::string &configFile)
	: AfiducialDetector(visionSysVector)
{
	//создаем экзмпляр класса параметров обнаружения опорных маркеров
	detectorParametrs = cv::aruco::DetectorParameters::create();
	setDicts(configFile);
}

ArucoDetector::~ArucoDetector()
{

}

void ArucoDetector::detectFidusial()
{
	for(unsigned int i = 0; i < visionSysVector.size(); i++)
	{
		visionSysVector[i]->inputVideo >> visionSysVector[i]->image;
		if (!visionSysVector[i]->image.empty())
		{
			for (unsigned int j = 0; j < dictionaryList.size(); j++)
			{
				cv::aruco::detectMarkers(visionSysVector[i]->image, dictionaryList[j], cornersList[j], idsList[j], detectorParametrs);
				#ifdef VISUALIZATION
					cv::aruco::drawDetectedMarkers(visionSysVector[i]->image, cornersList[j], idsList[j]);
				#endif
			}
			#ifdef VISUALIZATION
				
				VisualizationOnImage::showImage(visionSysVector[i]->image);
			#endif	
			
		}
		
	}
}

void ArucoDetector::updateParametrsFromCongig(const std::string &detectorParametrsConfigFile)
{
	
}

void ArucoDetector::setDicts(const std::string &dictionaryConfigFile)
{
	YAML::Node fs = YAML::LoadFile(dictionaryConfigFile);
	std::vector<int> 	markerDictionariesTypes = fs["marker_dictionaries_type"].as<std::vector<int>>();
	std::vector<int>	dictionariesSizes = fs["dictionaries_sizes"].as<std::vector<int>>();

	if (markerDictionariesTypes.size() != dictionariesSizes.size())
	{
		std::cout << "error check detector dictionary config size is not eqiual" << std::endl;
		return;
	}
	//	устанавливаем размерность вектора словарей
	dictionaryList.resize(dictionariesSizes.size());
	//	устанавливаем размерность вектора идентификаторов реперных маркеров
	idsList.resize(dictionariesSizes.size());
	//	устанавливаем размерность вектора угловых точек
	// 	угловые точки являютя вершинами реперного маркера (4 на каждый маркер т к он квадратный)
	cornersList.resize(dictionariesSizes.size());
	//устанавливаем словарь согласно конфигурации
	for (unsigned int i = 0; i < dictionariesSizes.size(); i++)
	{
		switch (markerDictionariesTypes[i])
		{
			case 4:
            	switch (dictionariesSizes[i])
				{
					case 50:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
						break;
					case 100:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
						break;
					case 250:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
						break;
					case 1000:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 5:
				switch (dictionariesSizes[i])
				{
					case 50:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
						break;
					case 100:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
						break;
					case 250:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
						break;
					case 1000:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 6:
				switch (dictionariesSizes[i])
				{
					case 50:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
						break;
					case 100:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
						break;
					case 250:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
						break;
					case 1000:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 7:
				switch (dictionariesSizes[i])
				{
					case 50:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
						break;
					case 100:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
						break;
					case 250:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
						break;
					case 1000:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 1:
				switch (dictionariesSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 16:
				switch (dictionariesSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 25:
				switch (dictionariesSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 3610:
				switch (dictionariesSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 3611:
				switch (dictionariesSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
		}
	}
}

AprilTagDetector::AprilTagDetector(std::vector<VisionSystem*>	&visionSysVector)
	: AfiducialDetector(visionSysVector)
{
	
}



AprilTagDetector::~AprilTagDetector()
{
	
}

void AprilTagDetector::detectFidusial()
{

}