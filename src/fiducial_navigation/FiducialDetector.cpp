#include "FiducialDetector.hpp"

AFiducialDetector::AFiducialDetector(std::vector<VisionSystem*>	&visionSysVector)
{
	this->visionSysVector = visionSysVector;
}

AFiducialDetector::~AFiducialDetector()
{
	//очищаем память
	for (auto & i : visionSysVector)
		delete i;
	
}

ArucoDetector::ArucoDetector(std::vector<VisionSystem*>	&visionSysVector, const std::string &configFile)
	: AFiducialDetector(visionSysVector)
{
	//создаем экзмпляр класса параметров обнаружения опорных маркеров
	detectorParameters_ = cv::aruco::DetectorParameters::create();
	setDicts(configFile);
}

void ArucoDetector::detectFiducial()
{
	detectorStatus = false;
	vec1i_t						markerIds;
	vec2CvPoint2f_t				markerCorners;
	
	cornersList.clear();
	idsList.clear();
	for (auto & i : visionSysVector)
	{
		ids.clear();
		corners.clear();
		// visionSysVector[i]->inputVideo >> visionSysVector[i]->image;
		if (!i->image.empty())
		{
			for (unsigned int j = 0; j < dictionaryList_.size(); j++)
			{
				cv::aruco::detectMarkers(i->image, dictionaryList_[j], markerCorners, markerIds, detectorParameters_);
				if (markerIds.size() != 0)
				{
					addIdsAndCorners(markerIds, markerCorners, j);
					detectorStatus = true;
				}
			}
			
		}
		idsList.push_back(ids);
		cornersList.push_back(corners);
	}
}

void AFiducialDetector::addIdsAndCorners(vec1i_t &markerIds, vec2CvPoint2f_t &corners, const unsigned int &index)
{
	for (unsigned int i = 0; i < markerIds.size(); i++)
	{
		ids.push_back(markerIds[i] + dictNumList[index]);
		this->corners.push_back(corners[i]);
	}
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
	dictionaryList_.resize(dictSizes.size());
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
			case ArucoBitSizes::ARUCO_4:
				dictNumList.push_back(0);
            	switch (dictSizes[i])
				{
					
					case DictSizes::MARKERS_50:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
						break;
					case DictSizes::MARKERS_100:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
						break;
					case DictSizes::MARKERS_250:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
						break;
					case DictSizes::MARKERS_1000:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::ARUCO_5:
				dictNumList.push_back(ArucoBitSizes::ARUCO_4 * dictArucoNumStep);
				switch (dictSizes[i])
				{
					case DictSizes::MARKERS_50:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
						break;
					case DictSizes::MARKERS_100:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
						break;
					case DictSizes::MARKERS_250:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
						break;
					case DictSizes::MARKERS_1000:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::ARUCO_6:
				dictNumList.push_back(2000);
				switch (dictSizes[i])
				{
					case DictSizes::MARKERS_50:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
						break;
                    case DictSizes::MARKERS_100:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
						break;
                    case DictSizes::MARKERS_250:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
						break;
					case DictSizes::MARKERS_1000:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::ARUCO_7:
				dictNumList.push_back(3000);
				switch (dictSizes[i])
				{
					case DictSizes::MARKERS_50:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
						break;
					case DictSizes::MARKERS_100:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
						break;
					case DictSizes::MARKERS_250:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
						break;
					case DictSizes::MARKERS_1000:
                        dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::ARUCO_ORIG:
				dictNumList.push_back(4000);
				switch (dictSizes[i])
				{
					case 0:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::APRILTAG_16:
				dictNumList.push_back(5000);
				switch (dictSizes[i])
				{
					case 0:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::APRILTAG_25:
				dictNumList.push_back(6000);
				switch (dictSizes[i])
				{
					case 0:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::APRILTAG_36H10:
				dictNumList.push_back(7000);
				switch (dictSizes[i])
				{
					case 0:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
			case ArucoBitSizes::APRILTAG_36H11:
				dictNumList.push_back(8000);
				switch (dictSizes[i])
				{
					case 0:
                      dictionaryList_[i] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
						break;
					default:
                    	std::cout << "Error dict_size for bit_size" << " is not correct"<< std::endl; 
				}
				break;
		}
	}
}
