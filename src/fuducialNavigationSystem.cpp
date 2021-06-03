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
		std::cout << "Error fiducial type is not defineded" << std::endl;

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
	axesSize = fs["axes_size"].as<double>();
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
				// соответствующие им вершины на плоскости изображения(индексы пикселей вершин в матрице пикселей изображения)
				Board->getBoardObjectAndImagePoints(fiducialDetector->idsList[i], fiducialDetector->cornersList[i]);

				if (!Board->objPointsMat.empty())
				{
					//решаем задачу перемещения и вращения относительно эталонного объекта (карты маркеров)
					cv::solvePnP(Board->objPointsMat, Board->imgPointsMat, fiducialDetector->visionSysVector[i]->cameraMatrix,
						fiducialDetector->visionSysVector[i]->distCoeffs, rvec, tvec, useExtrinsicGuess);

					posInMap = fiducialDetector->visionSysVector[i]->getCamPosition(rvec, tvec);
					std::cout << "posInMap" << std::endl;
					std::cout << posInMap << std::endl;
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector->visionSysVector[i]->image, fiducialDetector->cornersList[i],
																 fiducialDetector->idsList[i], rvec, tvec, fiducialDetector->visionSysVector[i]->cameraMatrix,
																fiducialDetector->visionSysVector[i]->distCoeffs, axesSize);
					#endif
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector->visionSysVector[i]->image,"source_" + std::to_string(i));
			Board->showPlotWithMarkers();
		#endif
	
	}

}

FiducialSlamNavigation::FiducialSlamNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector, const std::string &detectorConfigFile, const std::string &boardConfigFile)
	:	FiducialBoardNavigation(detectorType, visionSysVector, detectorConfigFile, boardConfigFile)
{
 	
	YAML::Node fs = YAML::LoadFile(boardConfigFile);
	markerSize = fs["marker_size"].as<double>();
	waitTimeBeforeMarkerAdd = fs["wait_time_before_marker_add"].as<double>();

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
					// для коррекции работы системы можно будет использовать один из маркеров для оценки положения в режиме построения карты
					//решаем задачу перемещения и вращения относительно эталонного объекта
					cv::solvePnP(Board->objPointsMat, Board->imgPointsMat, fiducialDetector->visionSysVector[i]->cameraMatrix,
						fiducialDetector->visionSysVector[i]->distCoeffs, rvec, tvec, useExtrinsicGuess);

					posInMap = fiducialDetector->visionSysVector[i]->getCamPosition(rvec, tvec);
					stateVector = posInMap;
					std::cout << "posInMap" << std::endl;
					std::cout << posInMap << std::endl;
					updateIdsDetectedTime(i);
					updateMap();
					
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector->visionSysVector[i]->image, fiducialDetector->cornersList[i],
																 fiducialDetector->idsList[i], rvec, tvec, fiducialDetector->visionSysVector[i]->cameraMatrix,
																fiducialDetector->visionSysVector[i]->distCoeffs, axesSize);
					#endif
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector->visionSysVector[i]->image,"source_" + std::to_string(i));
			Board->showPlotWithMarkers();
		#endif
	
	}
	//  удаляем информацию об обнаруженных маркерах вне карты, если повторно они не появились в поле зрения
	deliteNotUpdatedMarkers();
}

void	FiducialSlamNavigation::updateIdsDetectedTime(const unsigned int &cameraId)
{

	bool markerIsNew;
	// оценка положения найденных маркеров в СК камеры
	cv::aruco::estimatePoseSingleMarkers(Board->unKnownCorners, markerSize, fiducialDetector->visionSysVector[cameraId]->cameraMatrix,
											fiducialDetector->visionSysVector[cameraId]->distCoeffs, rvecs, tvecs);
	
	// для всех маркеров найденных вне карты на данном шаге алгоритма выполним следующие операции
	for (unsigned int i = 0; i < Board->unknownIds.size(); i++)
	{
		// выполним перевод маркера в СК карты
		VectorXd markerPoseInMap = transformPoseToMap(rvecs[i], tvecs[i], cameraId);
		// флаг присутствия маркера вне карты с таким же номером на предыдущих итерациях
		markerIsNew = true;
		// для всех маркеров вне карты 
		for (unsigned int j = 0; j < unknownIds.size(); j++)
		{
			// если маркер вне карты уже был найден на предыдущих итерациях то
			if (Board->unknownIds[i] == unknownIds[j])
			{
				// снимаем флаг присутствия маркера вне карты с таким же номером на предыдущих итерациях
				markerIsNew = false;
				// добавляем в вектор время за которое маркер находился в поле зрения системы
				unknownMarkerTime[j] += getLoopTime();
				// суммируем данные о положении маркера в системе координат карты с целью последующего осреднения
				unknownMarkerPose[j] += markerPoseInMap;//(unknownMarkerPose[j] + markerPoseInMap) / 2;
				// увеличиваем счетчик итераций за которые была получена оценка положения маркера вне карты
				unknownIdsIters[j] += 1;

				// debug
				VectorXd result = unknownMarkerPose[j] / unknownIdsIters[j];
				std::cout << "result" << std::endl;
				std::cout << result << std::endl;


			}
		}
		// если маркер вне карты обнаружен впервые то добавляем в массив новвые комаоненты для этого объекта
		if (markerIsNew)
		{
			unknownMarkerPose.push_back(markerPoseInMap);
			unknownIds.push_back(Board->unknownIds[i]);
			unknownMarkerTime.push_back(0);
			unknownIdsIters.push_back(1);
		}
	}
	
	

}
void FiducialSlamNavigation::deliteNotUpdatedMarkers()
{
	// флаг свидетельствующий об обновлении данных о маркере вне карты на данном шаге алгоритма
	bool markerInfoNotUpdate;
	// если в поле зрения системы присутствуют маркеры вне карты
	if (!unknownIds.empty())
	{
		// для всех маркеров вне карты проверяем факт их повторного обнаружения на данном шаге алгоритма
		for (unsigned int i = 0; i < unknownIds.size(); i++)
		{	
			// выставляем флаг отсутствия обновлений
			markerInfoNotUpdate = true;
			// для всех неизвестных маркеров на данном шаге алгоритма
			for (unsigned int j = 0; j < Board->unknownIds.size(); j++)
			{
				// если маркер был повторно найден в поле зрения системы
				if (Board->unknownIds[j] == unknownIds[i])
				{
					// снимаем флаг отсутствия обновлений
					markerInfoNotUpdate = false;
				}
				
			}
			// если маркер не был обнаружен повторно удаляем его из списка кандитатов для добавления в карту
			if (markerInfoNotUpdate)
			{
				unknownIds.erase(unknownIds.begin() + i);
				unknownMarkerTime.erase(unknownMarkerTime.begin() + i);
				unknownMarkerPose.erase(unknownMarkerPose.begin() + i);
				unknownIdsIters.erase(unknownIdsIters.begin() + i);
			}
		}
		
	}

}
VectorXd   FiducialSlamNavigation::transformPoseToMap(cv::Vec3d &rvecMarker, cv::Vec3d &tvecMarker, const unsigned int &cameraId)
{
	// получаем положение камеры в системе координат маркера обнаруженного вне карты
	VectorXd markerPoseCam = fiducialDetector->visionSysVector[cameraId]->getCamPosition(rvecMarker, tvecMarker);
	

	std::cout << "markerPoseCam" << std::endl;
	std::cout << markerPoseCam << std::endl;

	// получаем разницу углового положения между СК карты и СК маркеры
	// таким образом получаем угловое положение маркера в СК карты


	

	Eigen::Vector3d diff = CvMathOperations::getAngleDifferense(rvec,rvecMarker);
    std::cout << "------------------------" << std::endl;
	std::cout << "AnglesDiff" << std::endl;
	std::cout << diff << std::endl;
	std::cout << "------------------------" << std::endl;
	
	// markerPoseCam *= -1;
	// for (unsigned int i = 3; i < 6; i++)
	// {
	// 	if (markerPoseCam[i] > M_PI)
	// 		markerPoseCam[i] -= M_PI;
	// 	else if (markerPoseCam[i] < M_PI)
	// 		markerPoseCam[i] -= M_PI;
	// }
	markerPoseCam[3] = diff[0];
	markerPoseCam[4] = diff[1];
	markerPoseCam[5] = diff[2];

	
	std::cout << "markerPoseCamResAngle" << std::endl;
	std::cout << markerPoseCam << std::endl;

	// разворачиваем вектор положения камеры в СК маркера найденного вне карты на угловое положение маркера в СК карты
	Eigen::Vector3d markerPoseInMap = rotateVector(markerPoseCam);

	std::cout << "markerPoseInMap" << std::endl;
	std::cout << markerPoseInMap << std::endl;

	// получаем положение и ориентацию маркера в СК карты
	VectorXd markerPoseMap(6);
	markerPoseMap[0] = posInMap[0] - markerPoseInMap[0];
	markerPoseMap[1] = posInMap[1] - markerPoseInMap[1];
	markerPoseMap[2] = posInMap[2] - markerPoseInMap[2];
	markerPoseMap[3] = markerPoseCam[3];
	markerPoseMap[4] = markerPoseCam[4];
	markerPoseMap[5] = markerPoseCam[5];

	std::cout << "markerPoseMap" << std::endl;
	std::cout << markerPoseMap << std::endl;

	return markerPoseMap;

}

void	FiducialSlamNavigation::updateMap()
{
	// для всех маркеров обнаруженных вне карты
	for (unsigned int i = 0; i < unknownMarkerTime.size(); i++)
	{
		// если маркер находится вне карты более чем пороговое значение времени то добавляем его в карту
		if (unknownMarkerTime[i] >= waitTimeBeforeMarkerAdd)
		{
			// std::cout << "unknownMarkerPose" << std::endl;
			// std::cout << unknownMarkerPose[i] << std::endl;
			// усредняем результаты измерений положения маркера вне карты
			VectorXd result = unknownMarkerPose[i] / unknownIdsIters[i];

			std::cout << "finalResult" << std::endl;
			std::cout << result << std::endl;
			// добавляем маркер в карту
			Board->addMarkerToMap(unknownIds[i], markerSize, result);
			// очищаем массивы т к теперь этот маркер в карте
			unknownIds.erase(unknownIds.begin() + i);
			unknownMarkerTime.erase(unknownMarkerTime.begin() + i);
			unknownMarkerPose.erase(unknownMarkerPose.begin() + i);
			unknownIdsIters.erase(unknownIdsIters.begin() + i);
		}
	}
}

