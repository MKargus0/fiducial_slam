#include <fuducialNavigationSystem.hpp>


AfiducialNavigation::AfiducialNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector, const std::string &detectorConfigFile)
	// : AvisualNavigationSystem()
{
	// this->fiducialDetector = fiducialDetector;
	// YAML::Node fs = YAML::LoadFile(dictionaryConfigFile);

	if ( detectorType == 0)
		fiducialDetector = new ArucoDetector(visionSysVector,detectorConfigFile);
	else if ( detectorType == 1)
		fiducialDetector = new AprilTagDetector(visionSysVector,detectorConfigFile);
	else
		std::cout << "Error fiducial type is not definded" << std::endl;

}

AfiducialNavigation::~AfiducialNavigation()
{
	delete fiducialDetector;
}


FiducialBoardNavigation::FiducialBoardNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector,const std::string &detectorConfigFile, const std::string &boardConfigFile)
	: AfiducialNavigation(detectorType, visionSysVector, detectorConfigFile)
{
	// создаем доску с маркерами
	Board = new CastomMultipleBoard(fiducialDetector->idsList.size());

	YAML::Node fs = YAML::LoadFile(boardConfigFile);
	int markerCount  = fs["marker_count"].as<int>();
	std::vector<double> markerParams;
	int markerId;
	for (int i = 0; i < markerCount; i++)
    {
		markerParams =  fs["marker_" + std::to_string(i+1)].as<std::vector<double>>();

		if ((int)markerParams[0] == 4)
		{
			markerId = markerParams[1];
		}
		else if (markerParams[0] == 5)
		{
			markerId = markerParams[1] + 1000;
		}
		else if (markerParams[0] == 6)
		{
			markerId = markerParams[1] + 2000;
		}
		else if (markerParams[0] == 7)
		{
			markerId = markerParams[1] + 3000;
		}

		Board->addMarkerToMap(markerId, markerParams[8], markerParams[2], markerParams[3], markerParams[4],
							  markerParams[5], markerParams[6], markerParams[7]);


	}


	// std::vector<double> markerParams;
	// vec1CvPoint3f_t     markerCorners;
	// cv::Point3d			corn;
	// vec2CvPoint3f_t		rejectedCandidates;
	// vec1i_t				markerIds;
	// rejectedCandidates.resize(markerCount);
	// markerIds.resize(markerCount);

	// for (int i = 0; i < markerCount; i++)
    // {
	// 	markerParams =  fs["marker_" + std::to_string(i+1)].as<std::vector<double>>();
	// 	markerCorners.clear();
	// 	corn.x = markerParams[2] - markerParams[8] / 2;
    //    	corn.y = markerParams[3] + markerParams[8] / 2;
    //    	corn.z = markerParams[4];
	// 	CvMathOperations::rotateCorn(corn, markerParams[2], markerParams[3], markerParams[4],markerParams[5], markerParams[6], markerParams[7]);
	// 	markerCorners.push_back(corn);
	// 	corn.x = markerParams[2] + markerParams[8] / 2;
    //    	corn.y = markerParams[3] + markerParams[8] / 2;
    //    	corn.z = markerParams[4];
	// 	CvMathOperations::rotateCorn(corn, markerParams[2], markerParams[3], markerParams[4],markerParams[5], markerParams[6], markerParams[7]);
	// 	markerCorners.push_back(corn);
	// 	corn.x = markerParams[2] + markerParams[8] / 2;
    //    	corn.y = markerParams[3] - markerParams[8] / 2;
    //    	corn.z = markerParams[4];
	// 	CvMathOperations::rotateCorn(corn, markerParams[2], markerParams[3], markerParams[4],markerParams[5], markerParams[6], markerParams[7]);
	// 	markerCorners.push_back(corn);
	// 	corn.x = markerParams[2] - markerParams[8] / 2;
    //    	corn.y = markerParams[3] - markerParams[8] / 2;
    //    	corn.z = markerParams[4];
	// 	CvMathOperations::rotateCorn(corn, markerParams[2], markerParams[3], markerParams[4],markerParams[5], markerParams[6], markerParams[7]);
	// 	markerCorners.push_back(corn);
		
		
	// 	if ((int)markerParams[0] == 4)
	// 	{
	// 		markerIds[i] = markerParams[1];
	// 	}
	// 	else if (markerParams[0] == 5)
	// 	{
	// 		markerIds[i] = markerParams[1] + 1000;
	// 	}
	// 	else if (markerParams[0] == 6)
	// 	{
	// 		markerIds[i] = markerParams[1] + 2000;
	// 	}
	// 	else if (markerParams[0] == 7)
	// 	{
	// 		markerIds[i] = markerParams[1] + 3000;
	// 	}

	// 	rejectedCandidates[i] = markerCorners;
	// }

	// Board->addDataToBoard(rejectedCandidates, markerIds);

}

FiducialBoardNavigation::~FiducialBoardNavigation()
{
	delete Board;
}


void FiducialBoardNavigation::estimateState()
{
	fiducialDetector->detectFidusial();
	

	for (unsigned int i = 0; i < fiducialDetector->visionSysVector.size(); i++)
	{
		if (fiducialDetector->detectorStatus)
		{
				// получаем вершины маркеров на изображении и эталонные последовательности вершин
				// соответствующие им
				Board->getBoardObjectAndImagePoints(fiducialDetector->idsList[i], fiducialDetector->cornersList[i]);

				if (!Board->objPointsMat.empty())
				{
					//решаем задачу перемещения и вращения относительно эталонного объекта
					cv::solvePnP(Board->objPointsMat, Board->imgPointsMat, fiducialDetector->visionSysVector[i]->cameraMatrix,
						fiducialDetector->visionSysVector[i]->distCoeffs, rvec, tvec, useExtrinsicGuess);

					posInMap = fiducialDetector->visionSysVector[i]->getCamPosition(rvec, tvec);
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector->visionSysVector[i]->image, fiducialDetector->cornersList[i], fiducialDetector->idsList[i], rvec, tvec, fiducialDetector->visionSysVector[i]->cameraMatrix, fiducialDetector->visionSysVector[i]->distCoeffs);
					#endif
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector->visionSysVector[i]->image,"source_" + std::to_string(i));
		#endif
	
	}

}

FiducialSlamNavigation::FiducialSlamNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector, const std::string &detectorConfigFile, const std::string &boardConfigFile)
	:	FiducialBoardNavigation(detectorType, visionSysVector, detectorConfigFile, boardConfigFile)
{
 	waitTimeBeforeMarkerAdd = 0.3;
}

void	FiducialSlamNavigation::estimateState()
{
	fiducialDetector->detectFidusial();
	for (unsigned int i = 0; i < fiducialDetector->visionSysVector.size(); i++)
	{
		if (fiducialDetector->detectorStatus)
		{
				// получаем вершины маркеров на изображении и эталонные последовательности вершин
				// соответствующие им
				Board->getBoardObjectAndImagePoints(fiducialDetector->idsList[i], fiducialDetector->cornersList[i]);

				

				if (!Board->objPointsMat.empty())
				{
					//решаем задачу перемещения и вращения относительно эталонного объекта
					cv::solvePnP(Board->objPointsMat, Board->imgPointsMat, fiducialDetector->visionSysVector[i]->cameraMatrix,
						fiducialDetector->visionSysVector[i]->distCoeffs, rvec, tvec, useExtrinsicGuess);

					posInMap = fiducialDetector->visionSysVector[i]->getCamPosition(rvec, tvec);
					updateIdsDetectedTime(i);
					updateMap();
					
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector->visionSysVector[i]->image, fiducialDetector->cornersList[i], fiducialDetector->idsList[i], rvec, tvec, fiducialDetector->visionSysVector[i]->cameraMatrix, fiducialDetector->visionSysVector[i]->distCoeffs);
					#endif
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector->visionSysVector[i]->image,"source_" + std::to_string(i));
		#endif
	
	}
	deliteNotUpdatedMarkers();
}

void	FiducialSlamNavigation::updateIdsDetectedTime(const unsigned int &cameraId)
{

	bool markerIsNew;
	cv::aruco::estimatePoseSingleMarkers(Board->unKnownCorners, 0.61875, fiducialDetector->visionSysVector[cameraId]->cameraMatrix,
											fiducialDetector->visionSysVector[cameraId]->distCoeffs, rvecs, tvecs);
	for (unsigned int i = 0; i < Board->unknownIds.size(); i++)
	{
	
		
		VectorXd markerPoseInMap = transformPoseToMap(rvecs[i], tvecs[i], cameraId);
		
		markerIsNew = true;
		for (unsigned int j = 0; j < unknownIds.size(); j++)
		{
			if (Board->unknownIds[i] == unknownIds[j])
			{
				markerIsNew = false;
				unknownMarkerTime[j] += timeDelta;
				unknownMarkerPose[j] = (unknownMarkerPose[j] + markerPoseInMap) / 2;
			}
		}
		if (markerIsNew)
		{
			unknownMarkerPose.push_back(markerPoseInMap);
			unknownIds.push_back(Board->unknownIds[i]);
			unknownMarkerTime.push_back(0);
		}
	}
	//  удаляем информацию об обнаруженных маркерах вне карты если она не обновилась
	

}
void FiducialSlamNavigation::deliteNotUpdatedMarkers()
{
	bool markerInfoNotUpdate;
	if (!unknownIds.empty())
	{
		
		for (unsigned int i = 0; i < unknownIds.size(); i++)
		{	
			markerInfoNotUpdate = true;
			for (unsigned int j = 0; j < Board->unknownIds.size(); j++)
			{
				if (Board->unknownIds[j] == unknownIds[i])
				{
					markerInfoNotUpdate = false;
				}
				
			}
			if (markerInfoNotUpdate)
			{
				unknownIds.erase(unknownIds.begin() + i);
				unknownMarkerTime.erase(unknownMarkerTime.begin() + i);
				unknownMarkerPose.erase(unknownMarkerPose.begin() + i);
			}
		}
		
	}

}
VectorXd   FiducialSlamNavigation::transformPoseToMap(cv::Vec3d &rvec, cv::Vec3d &tvec, const unsigned int &cameraId)
{
	VectorXd markerPoseCam = fiducialDetector->visionSysVector[cameraId]->getCamPosition(rvec, tvec);
	// получаем разницу углового положения между СК карты и СК маркеры
	// таким образом получаем угловое положение маркера в СК карты
	markerPoseCam[3] = posInMap[3] - markerPoseCam[3]; 
	markerPoseCam[4] = posInMap[4] - markerPoseCam[4];
	markerPoseCam[4] = posInMap[5] - markerPoseCam[5];
	// разворачиваем вектор положения камеры на угловое положение маркера в СК карты
	Eigen::Vector3d markerPoseInMap = rotateVector(markerPoseCam);
	// получаем положение и ориентацию маркера в СК карты
	VectorXd markerPoseMap(6);
	markerPoseMap[0] = posInMap[0] - markerPoseInMap[0];
	markerPoseMap[1] = posInMap[1] - markerPoseInMap[1];
	markerPoseMap[2] = posInMap[2] - markerPoseInMap[2];
	markerPoseMap[3] = markerPoseCam[3];
	markerPoseMap[4] = markerPoseCam[4];
	markerPoseMap[5] = markerPoseCam[5];

	return markerPoseMap;

}

void	FiducialSlamNavigation::updateMap()
{
	for (unsigned int i = 0; i < unknownMarkerTime.size(); i++)
	{
		if (unknownMarkerTime[i] >= waitTimeBeforeMarkerAdd)
		{
			// std::cout << "unknownMarkerPose" << std::endl;
			// std::cout << unknownMarkerPose[i] << std::endl;
			Board->addMarkerToMap(unknownIds[i], 1, unknownMarkerPose[i]);
		}
	}
}

void FiducialSlamNavigation::setDeltaTime(double &timeDelta)
{
	this->timeDelta = timeDelta;
}
