#include "FiducialNavigationSystem.hpp"


AfiducialNavigation::AfiducialNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector,
										 const std::string &detectorConfigFile)
{
	stateVector_.resize(6);
	if ( detectorType == 0)
      fiducialDetector_ = new ArucoDetector(visionSysVector, detectorConfigFile);
	else
		std::cout << "Error fiducial type is not defineded" << std::endl;

	// TODO add apriltag
}

AfiducialNavigation::~AfiducialNavigation()
{
	delete fiducialDetector_;
}


FiducialBoardNavigation::FiducialBoardNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector,
												const std::string &detectorConfigFile, const std::string &boardConfigFile)
	: AfiducialNavigation(detectorType, visionSysVector, detectorConfigFile)
{
	// создаем доску с маркерами
	board_ = new CustomMultipleBoard(fiducialDetector_->idsList.size());

	YAML::Node fs = YAML::LoadFile(boardConfigFile);
	int markerCount  = fs["marker_count"].as<int>();
  axesSize_ = fs["axes_size"].as<double>();
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

		board_->addMarkerToMap(markerId, markerParams[8], markerParams[2],
                               markerParams[3], markerParams[4], markerParams[5],
                               markerParams[6], markerParams[7]);

	}
}

FiducialBoardNavigation::~FiducialBoardNavigation()
{
	delete board_;
}

int FiducialBoardNavigation::estimateState()
{
	// находим реперные маркеры
  fiducialDetector_->detectFiducial();
	
	int status = 0;
	for (unsigned int i = 0; i < fiducialDetector_->visionSysVector.size(); i++)
	{
		if (fiducialDetector_->detectorStatus)
		{		
				// получаем вершины маркеров на изображении и эталонные последовательности вершин
				// соответствующие им вершины на плоскости изображения(индексы пикселей вершин в матрице пикселей изображения)
				board_->getBoardObjectAndImagePoints(fiducialDetector_->idsList[i], fiducialDetector_->cornersList[i]);
				if (!board_->objPointsMat.empty())
				{
					//решаем задачу перемещения и вращения относительно эталонного объекта (карты маркеров)
					cv::solvePnP(board_->objPointsMat, board_->imgPointsMat, fiducialDetector_->visionSysVector[i]->cameraMatrix,
                                 fiducialDetector_->visionSysVector[i]->distCoeffs, rVec_, tVec_, useExtrinsicGuess_);
					// получаем положение камеры в СК карты
					posInMap_ = fiducialDetector_->visionSysVector[i]->getCamPosition(rVec_, tVec_);
					stateVector_ = posInMap_;
					#ifdef DEBUG
					std::cout << "posInMap_" << std::endl;
					std::cout << posInMap_ << std::endl;
					#endif
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector_->visionSysVector[i]->image, fiducialDetector_->cornersList[i],
                                                                 fiducialDetector_->idsList[i], rVec_, tVec_, fiducialDetector_->visionSysVector[i]->cameraMatrix,
                                                                 fiducialDetector_->visionSysVector[i]->distCoeffs, axesSize_);
					#endif
					status = 1;
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector_->visionSysVector[i]->image, "source_" + std::to_string(i));
		#endif

		#ifdef USE_PLOTTER
			board_->showPlotWithMarkers();
		#endif
	
	}

	return (status);

}

FiducialSlamNavigation::FiducialSlamNavigation(unsigned int detectorType, std::vector<VisionSystem*> visionSysVector,
												const std::string &detectorConfigFile, const std::string &boardConfigFile)
	:	FiducialBoardNavigation(detectorType, visionSysVector, detectorConfigFile, boardConfigFile)
{
 	
	YAML::Node fs = YAML::LoadFile(boardConfigFile);
  markerSize_ = fs["marker_size"].as<double>();
  waitTimeBeforeMarkerAdd_ = fs["wait_time_before_marker_add"].as<double>();

}

int	FiducialSlamNavigation::estimateState()
{
	int status = 0;
  fiducialDetector_->detectFiducial();
	for (unsigned int i = 0; i < fiducialDetector_->visionSysVector.size(); i++)
	{
		if (fiducialDetector_->detectorStatus)
		{
				// получаем вершины маркеров на изображении и эталонные последовательности вершин
				// соответствующие им
				board_->getBoardObjectAndImagePoints(fiducialDetector_->idsList[i], fiducialDetector_->cornersList[i]);

				if (!board_->objPointsMat.empty())
				{
					// для коррекции работы системы можно будет использовать один из маркеров для оценки положения в режиме построения карты
					//решаем задачу перемещения и вращения относительно эталонного объекта
					cv::solvePnP(board_->objPointsMat, board_->imgPointsMat, fiducialDetector_->visionSysVector[i]->cameraMatrix,
                                 fiducialDetector_->visionSysVector[i]->distCoeffs, rVec_, tVec_, useExtrinsicGuess_);

                  posInMap_ = fiducialDetector_->visionSysVector[i]->getCamPosition(rVec_, tVec_);
					stateVector_ = posInMap_;
					#ifdef DEBUG
					std::cout << "posInMap_" << std::endl;
					std::cout << posInMap_ << std::endl;
					#endif
					updateIdsDetectedTime(i); 
					updateMap();
					
					#ifdef VISUALIZATION
						VisualizationOnImage::drawMarkersAndAxes(fiducialDetector_->visionSysVector[i]->image, fiducialDetector_->cornersList[i],
                                                                 fiducialDetector_->idsList[i], rVec_, tVec_,
                                                                 fiducialDetector_->visionSysVector[i]->cameraMatrix,
                                                                 fiducialDetector_->visionSysVector[i]->distCoeffs, axesSize_);
					#endif
					status = 1;
				}
		}

		#ifdef VISUALIZATION
			VisualizationOnImage::showImage(fiducialDetector_->visionSysVector[i]->image, "source_" + std::to_string(i));
			
			#ifdef USE_PLOTTER
			board_->showPlotWithMarkers();
			#endif
			
		#endif

		
	
	}
	//  удаляем информацию об обнаруженных маркерах вне карты, если повторно они не появились в поле зрения
  deleteNotUpdatedMarkers();
	return (status);
}

void	FiducialSlamNavigation::updateIdsDetectedTime(const unsigned int &cameraId)
{

	bool markerIsNew;
	// оценка положения найденных маркеров в СК камеры
	cv::aruco::estimatePoseSingleMarkers(board_->unKnownCorners, markerSize_, fiducialDetector_->visionSysVector[cameraId]->cameraMatrix,
                                         fiducialDetector_->visionSysVector[cameraId]->distCoeffs, rVecs_, tVecs_);
	
	// для всех маркеров найденных вне карты на данном шаге алгоритма выполним следующие операции
	for (unsigned int i = 0; i < board_->unknownIds.size(); i++)
	{
		// выполним перевод маркера в СК карты
		VectorXd_t markerPoseInMap = transformPoseToMap(rVecs_[i], tVecs_[i], cameraId);
		// флаг присутствия маркера вне карты с таким же номером на предыдущих итерациях
		markerIsNew = true;
		// для всех маркеров вне карты 
		for (unsigned int j = 0; j < unknownIds_.size(); j++)
		{
			// если маркер вне карты уже был найден на предыдущих итерациях то
			if (board_->unknownIds[i] == unknownIds_[j])
			{
				// снимаем флаг присутствия маркера вне карты с таким же номером на предыдущих итерациях
				markerIsNew = false;
				// добавляем в вектор время за которое маркер находился в поле зрения системы
				unknownMarkerTime[j] += getLoopTime();
				// суммируем данные о положении маркера в системе координат карты с целью последующего осреднения
				// unknownMarkerPose[j] += markerPoseInMap;//(unknownMarkerPose[j] + markerPoseInMap) / 2;
				unknownMarkerPoseX[j].push_back(markerPoseInMap[0]);
				unknownMarkerPoseY[j].push_back(markerPoseInMap[1]);
				unknownMarkerPoseZ[j].push_back(markerPoseInMap[2]);
				unknownMarkerPoseRoll[j].push_back({markerPoseInMap[3]});
				unknownMarkerPosePitch[j].push_back({markerPoseInMap[4]});
				unknownMarkerPoseYaw[j].push_back({markerPoseInMap[5]});
				// увеличиваем счетчик итераций за которые была получена оценка положения маркера вне карты
				unknownIdsIters_[j] += 1;

				#ifdef DEBUG
				std::cout << "result" << std::endl;
				std::cout << result << std::endl;
				#endif

			}
		}
		// если маркер вне карты обнаружен впервые то добавляем в массив новвые комаоненты для этого объекта
		if (markerIsNew)
		{
			// unknownMarkerPose.push_back(markerPoseInMap);
			unknownMarkerPoseX.push_back({markerPoseInMap[0]});
			unknownMarkerPoseY.push_back({markerPoseInMap[1]});
			unknownMarkerPoseZ.push_back({markerPoseInMap[2]});
			unknownMarkerPoseRoll.push_back({markerPoseInMap[3]});
			unknownMarkerPosePitch.push_back({markerPoseInMap[4]});
			unknownMarkerPoseYaw.push_back({markerPoseInMap[5]});
			unknownIds_.push_back(board_->unknownIds[i]);
			unknownMarkerTime.push_back(0);
			unknownIdsIters_.push_back(1);
		}
	}
}

void FiducialSlamNavigation::deleteNotUpdatedMarkers()
{
	// флаг свидетельствующий об обновлении данных о маркере вне карты на данном шаге алгоритма
	bool markerInfoNotUpdate;
	// если в поле зрения системы присутствуют маркеры вне карты
	if (!unknownIds_.empty())
	{
		// для всех маркеров вне карты проверяем факт их повторного обнаружения на данном шаге алгоритма
		for (unsigned int i = 0; i < unknownIds_.size(); i++)
		{	
			// выставляем флаг отсутствия обновлений
			markerInfoNotUpdate = true;
			// для всех неизвестных маркеров на данном шаге алгоритма
			for (unsigned int j = 0; j < board_->unknownIds.size(); j++)
			{
				// если маркер был повторно найден в поле зрения системы
				if (board_->unknownIds[j] == unknownIds_[i])
				{
					// снимаем флаг отсутствия обновлений
					markerInfoNotUpdate = false;
				}
				
			}
			// если маркер не был обнаружен повторно удаляем его из списка кандитатов для добавления в карту
			if (markerInfoNotUpdate)
			{
				unknownIds_.erase(unknownIds_.begin() + i);
				unknownMarkerTime.erase(unknownMarkerTime.begin() + i);
				// unknownMarkerPose.erase(unknownMarkerPose.begin() + i);
				unknownMarkerPoseX.erase(unknownMarkerPoseX.begin() + i);
				unknownMarkerPoseY.erase(unknownMarkerPoseY.begin() + i);
				unknownMarkerPoseZ.erase(unknownMarkerPoseZ.begin() + i);
				unknownMarkerPoseRoll.erase(unknownMarkerPoseRoll.begin() + i);
				unknownMarkerPosePitch.erase(unknownMarkerPosePitch.begin() + i);
				unknownMarkerPoseYaw.erase(unknownMarkerPoseYaw.begin() + i);
				unknownIdsIters_.erase(unknownIdsIters_.begin() + i);
			}
		}
		
	}

}

VectorXd_t   FiducialSlamNavigation::transformPoseToMap(cv::Vec3d &rvecMarker, cv::Vec3d &tvecMarker, const unsigned int &cameraId)
{
	// получаем положение камеры в системе координат маркера обнаруженного вне карты
	VectorXd_t markerPoseCam = fiducialDetector_->visionSysVector[cameraId]->getCamPosition(rvecMarker, tvecMarker);
	
	#ifdef DEBUG
	std::cout << "markerPoseCam" << std::endl;
	std::cout << markerPoseCam << std::endl;
	#endif

	// получаем разницу углового положения между СК карты и СК маркеры
	// таким образом получаем угловое положение маркера в СК карты
	Eigen::Vector3d diff = CvMathOperations::getAngleDifferense(rVec_, rvecMarker);
	#ifdef DEBUG
    std::cout << "------------------------" << std::endl;
	std::cout << "AnglesDiff" << std::endl;
	std::cout << diff << std::endl;
	std::cout << "------------------------" << std::endl;
	#endif
	
	markerPoseCam[3] = diff[0];
	markerPoseCam[4] = diff[1];
	markerPoseCam[5] = diff[2];

	#ifdef DEBUG
	std::cout << "markerPoseCamResAngle" << std::endl;
	std::cout << markerPoseCam << std::endl;
	#endif

	// разворачиваем вектор положения камеры в СК маркера найденного вне карты на угловое положение маркера в СК карты
	Eigen::Vector3d markerPoseInMap = rotateVector(markerPoseCam);

	#ifdef DEBUG
	std::cout << "markerPoseInMap" << std::endl;
	std::cout << markerPoseInMap << std::endl;
	#endif

	// получаем положение и ориентацию маркера в СК карты
	VectorXd_t markerPoseMap(6);
	markerPoseMap[0] = posInMap_[0] - markerPoseInMap[0];
	markerPoseMap[1] = posInMap_[1] - markerPoseInMap[1];
	markerPoseMap[2] = posInMap_[2] - markerPoseInMap[2];
	markerPoseMap[3] = markerPoseCam[3];
	markerPoseMap[4] = markerPoseCam[4];
	markerPoseMap[5] = markerPoseCam[5];

	#ifdef DEBUG
	std::cout << "markerPoseMap" << std::endl;
	std::cout << markerPoseMap << std::endl;
	#endif

	return (markerPoseMap);
}

void	FiducialSlamNavigation::updateMap()
{
	// для всех маркеров обнаруженных вне карты
	for (unsigned int i = 0; i < unknownMarkerTime.size(); i++)
	{
		// если маркер находится вне карты более чем пороговое значение времени то добавляем его в карту
		if (unknownMarkerTime[i] >= waitTimeBeforeMarkerAdd_)
		{
			// применяем медианный фильтр
			std::sort(unknownMarkerPoseX[i].begin(), unknownMarkerPoseX[i].end());
			std::sort(unknownMarkerPoseY[i].begin(), unknownMarkerPoseY[i].end());
			std::sort(unknownMarkerPoseZ[i].begin(), unknownMarkerPoseZ[i].end());
			std::sort(unknownMarkerPoseRoll[i].begin(), unknownMarkerPoseRoll[i].end());
			std::sort(unknownMarkerPosePitch[i].begin(), unknownMarkerPosePitch[i].end());
			std::sort(unknownMarkerPoseYaw[i].begin(), unknownMarkerPoseYaw[i].end());

			VectorXd_t result(6);
			result[0] = unknownMarkerPoseX[i][ceil(unknownMarkerPoseX[i].size()/2)-1];
			result[1] = unknownMarkerPoseY[i][ceil(unknownMarkerPoseY[i].size()/2)-1];
			result[2] = unknownMarkerPoseZ[i][ceil(unknownMarkerPoseZ[i].size()/2)-1];
			result[3] = unknownMarkerPoseRoll[i][ceil(unknownMarkerPoseRoll[i].size()/2)-1];
			result[4] = unknownMarkerPosePitch[i][ceil(unknownMarkerPosePitch[i].size()/2)-1];
			result[5] = unknownMarkerPoseYaw[i][ceil(unknownMarkerPoseYaw[i].size()/2)-1];
			// добавляем маркер в карту
			board_->addMarkerToMap(unknownIds_[i], markerSize_, result);
			// очищаем массивы т к теперь этот маркер в карте
			unknownIds_.erase(unknownIds_.begin() + i);
			unknownMarkerTime.erase(unknownMarkerTime.begin() + i);
			// unknownMarkerPose.erase(unknownMarkerPose.begin() + i);
			unknownMarkerPoseX.erase(unknownMarkerPoseX.begin() + i);
			unknownMarkerPoseY.erase(unknownMarkerPoseY.begin() + i);
			unknownMarkerPoseZ.erase(unknownMarkerPoseZ.begin() + i);
			unknownMarkerPoseRoll.erase(unknownMarkerPoseRoll.begin() + i);
			unknownMarkerPosePitch.erase(unknownMarkerPosePitch.begin() + i);
			unknownMarkerPoseYaw.erase(unknownMarkerPoseYaw.begin() + i);
			unknownIdsIters_.erase(unknownIdsIters_.begin() + i);
		}
	}
}

