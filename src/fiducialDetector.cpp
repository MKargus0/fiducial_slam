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
	detectorStatus = false;
	vec1i_t						markerIds;
	vec2CvPoint2f_t				markerCorners;
	
	cornersList.clear();
	idsList.clear();
	for(unsigned int i = 0; i < visionSysVector.size(); i++)
	{
		ids.clear();
		corners.clear();
		visionSysVector[i]->inputVideo >> visionSysVector[i]->image;
		if (!visionSysVector[i]->image.empty())
		{
			for (unsigned int j = 0; j < dictionaryList.size(); j++)
			{
				cv::aruco::detectMarkers(visionSysVector[i]->image, dictionaryList[j], markerCorners, markerIds, detectorParametrs);
				if (markerIds.size() != 0)
				{
					addIdsAndCorners(markerIds, markerCorners, j);
					detectorStatus = true;
				
					// после нужно андистортнуть точки для этой камеры чтобы потом вызывать единый солвПнП 
					// #ifdef VISUALIZATION
					// 	cv::aruco::drawDetectedMarkers(visionSysVector[i]->image, markerCorners, markerIds);
					// #endif
				}
			}
			// #ifdef VISUALIZATION
				
			// 	VisualizationOnImage::showImage(visionSysVector[i]->image,"source_" + std::to_string(i));
			// #endif	
			
		}
		idsList.push_back(ids);
		cornersList.push_back(corners);
	}
}

void AfiducialDetector::addIdsAndCorners(vec1i_t &markerIds, vec2CvPoint2f_t &corners, const unsigned int &index)
{
	for (unsigned int i = 0; i < markerIds.size(); i++)
	{
		ids.push_back(markerIds[i] + dictNumList[index]);
		this->corners.push_back(corners[i]);
	}
}

void ArucoDetector::updateParametrsFromCongig(const std::string &detectorParametrsConfigFile)
{
	
}

void ArucoDetector::setDicts(const std::string &dictionaryConfigFile)
{
	YAML::Node fs = YAML::LoadFile(dictionaryConfigFile);
	dictsMarkersBitSize = fs["marker_dictionaries_type"].as<std::vector<int>>();
	dictSizes = fs["dictionaries_sizes"].as<std::vector<int>>();

	if (dictsMarkersBitSize.size() != dictSizes.size())
	{
		std::cout << "error check detector dictionary config size is not eqiual" << std::endl;
		return;
	}
	//	устанавливаем размерность вектора словарей
	dictionaryList.resize(dictSizes.size());
	//	устанавливаем размерность вектора идентификаторов реперных маркеров
	idsList.resize(dictSizes.size());
	//	устанавливаем размерность вектора угловых точек
	// 	угловые точки являютя вершинами реперного маркера (4 на каждый маркер т к он квадратный)
	cornersList.resize(dictSizes.size());
	//устанавливаем словарь согласно конфигурации
	for (unsigned int i = 0; i < dictSizes.size(); i++)
	{
		switch (dictsMarkersBitSize[i])
		{
			case 4:
				dictNumList.push_back(0);
            	switch (dictSizes[i])
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
				dictNumList.push_back(1000);
				switch (dictSizes[i])
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
				dictNumList.push_back(2000);
				switch (dictSizes[i])
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
				dictNumList.push_back(3000);
				switch (dictSizes[i])
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
				dictNumList.push_back(4000);
				switch (dictSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 16:
				dictNumList.push_back(5000);
				switch (dictSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 25:
				dictNumList.push_back(6000);
				switch (dictSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 3610:
				dictNumList.push_back(7000);
				switch (dictSizes[i])
				{
					case 0:
						dictionaryList[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case 3611:
				dictNumList.push_back(8000);
				switch (dictSizes[i])
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

AprilTagDetector::AprilTagDetector(std::vector<VisionSystem*>	&visionSysVector, const std::string &configFile)
	: AfiducialDetector(visionSysVector)
{
	
}



AprilTagDetector::~AprilTagDetector()
{
	
}

void AprilTagDetector::detectFidusial()
{

}