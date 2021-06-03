#include <customBoard.hpp>


CastomMultipleBoard::CastomMultipleBoard(const unsigned int &dictionaryCount)
{
	// objPoints.resize(dictionaryCount);
	// ids.resize(dictionaryCount);
	objPoints.clear();
	ids.clear();
	this->dictionaryCount = dictionaryCount;
	#ifdef VISUALIZATION
		plot.size(800,800);
		plot.legend().hide();
		plot.xlabel("x");
		plot.ylabel("y");
		plot.zlabel("z");
		plot.zrange(-0.3, 1);
		plot.xrange(-1, 1);
		plot.yrange(-1, 1);
		plot.autoclean(false);
		markerPlotIndex = 0;
	#endif
}


// void CastomMultipleBoard::addDataToBoard(vec3CvPoint3f_t &objPoints, vec2i_t &ids)
// {
// 	// add checks for dimentions of input data

// 	vec3CvPoint3f_t obj_points_vector;
// 	obj_points_vector.resize(dictionaryCount);
// 	for (unsigned int i = 0; i < objPoints.size(); i++)
// 	{
// 		// objPoints[i] = (cv::InputArrayOfArrays)(objPoints[i]);
// 		setCorners(objPoints[i], obj_points_vector[i]);

// 	}
// 	this->ids = ids;
// 	this->objPoints = obj_points_vector;
	
// }

void CastomMultipleBoard::addDataToBoard(cv::InputArrayOfArrays &objPoints, cv::InputArray &ids)
{
	CV_Assert(objPoints.total() == ids.total());
    CV_Assert(objPoints.type() == CV_32FC3 || objPoints.type() == CV_32FC1);
	std::vector< std::vector< cv::Point3f > > obj_points_vector;

    for (unsigned int i = 0; i < objPoints.total(); i++)
	{
        std::vector<cv::Point3f> corners;
    	cv::Mat corners_mat = objPoints.getMat(i);

        if(corners_mat.type() == CV_32FC1)
            corners_mat = corners_mat.reshape(3);
        // CV_Assert(corners_mat.total() == 4);

        for (int j = 0; j < 4; j++)
            corners.push_back(corners_mat.at<cv::Point3f>(j));
        
        obj_points_vector.push_back(corners);
    }

	ids.copyTo(this->ids);
	this->objPoints = obj_points_vector;

}

void CastomMultipleBoard::updateDataMap(const unsigned int &markerId,vec1CvPoint2f_t &markerCorners)
{
	unknownIds.push_back(markerId);
	unKnownCorners.push_back(markerCorners);
}

void CastomMultipleBoard::getBoardObjectAndImagePoints(cv::InputArray &detectedIds, cv::InputArrayOfArrays &detectedCorners)
{
	//очишаем данные о маркерах не входящих в карту
	unknownIds.clear();
	unKnownCorners.clear();
	// получаем размер найденных маркеров для конкретного словаря
	size_t nDetectedMarkers = detectedIds.total();
	// инициализируем вектор угловых точек (эталонный)
	vec1CvPoint3f_t  objPnts;
	// тоже самое для угловых точек(найденных на изображении)
	vec1CvPoint2f_t  imgPnts;
	// выставляем размерность согласно найденным для конкретного словаря маркерам
	objPnts.reserve(nDetectedMarkers);
	imgPnts.reserve(nDetectedMarkers);

	// для всех найденных маркеров у i-го словаря
	for (unsigned int i = 0; i < nDetectedMarkers; i++)
	{	
		// получаем текущий номер
		int currentId = detectedIds.getMat().ptr< int >(0)[i];
		bool inMap = false;
		// осуществляем поиск внутри доски пробуя найти маркер с таким же номером в базе
		for (unsigned int j = 0; j < ids.size(); j++)
		{

			if(currentId == ids[j])
			{
				inMap = true;
				//если нашли то добавляем данные в массив эталонной последовательности и 
				// массив точек полученный на изображении в необходимом типе данных
				for (int p = 0; p < 4; p++)
				{
					objPnts.push_back(objPoints[j][p]);
                	imgPnts.push_back(detectedCorners.getMat(i).ptr< cv::Point2f >(0)[p]);
				}
				break;
			}
		}

		if (!inMap)
		{
			vec1CvPoint2f_t detCorners; 
			for (int p = 0; p < 4; p++)
			{
				detCorners.push_back(detectedCorners.getMat(i).ptr< cv::Point2f >(0)[p]);
			}

			// std::cout << "detCorners" << std::endl;
			// std::cout << detCorners << std::endl;
			updateDataMap(currentId, detCorners);
			std::cout << "marker " << currentId << " no find in the map" << std::endl;
		}

	}

	// копируем в матрицу
    cv::Mat(objPnts).copyTo(objPointsMat);
    cv::Mat(imgPnts).copyTo(imgPointsMat);

	// std::cout << "objPointsMat" << std::endl;
	// std::cout << objPointsMat << std::endl;
	// std::cout << "imgPointsMat" << std::endl;
	// std::cout << imgPointsMat << std::endl;
	
}


void CastomMultipleBoard::addMarkerToBoard(cv::InputArray &rejected_candidate, const unsigned int &markerId)
{
	std::vector<cv::Point3f> corners;
	cv::Mat corners_mat = rejected_candidate.getMat();

	if(corners_mat.type() == CV_32FC1)
        corners_mat = corners_mat.reshape(3);

	for (int k = 0; k < 4; k++) 
        corners.push_back(corners_mat.at<cv::Point3f>(k));
			
	this->objPoints.push_back(corners);
	this->ids.push_back(markerId);
}

void CastomMultipleBoard::addMarkerToMap(const unsigned int &markerId, const double &markerSize, double &boardX, double &boardY,
							const double &boardZ,const double &roll, const double &pitch,const double &yaw)
{
	
	vec1CvPoint3f_t     markerCorners;
	cv::Point3d			corn;
	markerCorners.clear();
	corn.x = boardX - markerSize / 2;
    corn.y = boardY + markerSize / 2;
    corn.z = boardZ;
	corn = CvMathOperations::rotateCorn(corn, boardX, boardY, boardZ, roll, pitch, yaw);
	markerCorners.push_back(corn);
	corn.x = boardX + markerSize / 2;
    corn.y = boardY + markerSize / 2;
    corn.z = boardZ;
	corn = CvMathOperations::rotateCorn(corn, boardX, boardY, boardZ, roll, pitch, yaw);
	markerCorners.push_back(corn);
	corn.x = boardX + markerSize / 2;
    corn.y = boardY - markerSize / 2;
    corn.z = boardZ;
	corn = CvMathOperations::rotateCorn(corn, boardX, boardY, boardZ, roll, pitch, yaw);
	markerCorners.push_back(corn);
	corn.x = boardX - markerSize / 2;
    corn.y = boardY - markerSize / 2;
    corn.z = boardZ;
	corn = CvMathOperations::rotateCorn(corn, boardX, boardY, boardZ, roll, pitch, yaw);
	markerCorners.push_back(corn);

	addMarkerToBoard(markerCorners, markerId);
}

void CastomMultipleBoard::addMarkerToMap(const unsigned int &markerId, const double &markerSize, VectorXd &markerPose)
{
	vec1CvPoint3f_t     markerCorners;
	cv::Point3d			corn;
	markerCorners.clear();
	corn.x = markerPose[0] - markerSize / 2;
    corn.y = markerPose[1] + markerSize / 2;
    corn.z = markerPose[2];
	corn = CvMathOperations::rotateCorn(corn, markerPose[0], markerPose[1], markerPose[2], markerPose[3], markerPose[4], markerPose[5]);
	markerCorners.push_back(corn);
	corn.x = markerPose[0] + markerSize / 2;
    corn.y = markerPose[1] + markerSize / 2;
    corn.z = markerPose[2];
	corn = CvMathOperations::rotateCorn(corn, markerPose[0], markerPose[1], markerPose[2], markerPose[3], markerPose[4], markerPose[5]);
	markerCorners.push_back(corn);
	corn.x = markerPose[0] + markerSize / 2;
    corn.y = markerPose[1] - markerSize / 2;
    corn.z = markerPose[2];
	corn = CvMathOperations::rotateCorn(corn, markerPose[0], markerPose[1], markerPose[2], markerPose[3], markerPose[4], markerPose[5]);
	markerCorners.push_back(corn);
	corn.x = markerPose[0] - markerSize / 2;
    corn.y = markerPose[1] - markerSize / 2;
    corn.z = markerPose[2];
	corn = CvMathOperations::rotateCorn(corn, markerPose[0], markerPose[1], markerPose[2], markerPose[3], markerPose[4], markerPose[5]);
	markerCorners.push_back(corn);
	addMarkerToBoard(markerCorners, markerId);
}
#ifdef	VISUALIZATION
void CastomMultipleBoard::showPlotWithMarkers()
{
	dataX.clear();
	dataY.clear();
	dataZ.clear();

	std::vector<double> lineX;
	std::vector<double> lineY;
	std::vector<double> lineZ;
	lineX.clear();
	lineY.clear();
	lineZ.clear();
	for (int i = markerPlotIndex; i < ids.size(); i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			dataX.push_back(objPoints[i][j].x);
			dataY.push_back(objPoints[i][j].y);
			dataZ.push_back(objPoints[i][j].z);
			// plot.drawPoints();
		}
		
		// lineX.push_back(objPoints[i][0].x);
		// lineY.push_back(objPoints[i][0].y);
		// lineZ.push_back(objPoints[i][0].z);
		// lineX.push_back(objPoints[i][1].x);
		// lineY.push_back(objPoints[i][1].y);
		// lineZ.push_back(objPoints[i][1].z);
		// plot.drawCurve(lineX,lineY,lineZ);
		// plot.show(true);
		markerPlotIndex++;
	}
	plot.drawPoints(dataX, dataY, dataZ).pointType(2);
	plot.show(true);
}	
#endif