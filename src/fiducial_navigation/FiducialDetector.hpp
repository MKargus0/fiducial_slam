#ifndef 	FIDUCIAL_SLAM_FIDUCIAL_NAVIGATION_FIDUCIAL_DETECTOR_HPP
#define 	FIDUCIAL_SLAM_FIDUCIAL_NAVIGATION_FIDUCIAL_DETECTOR_HPP

#include 	<opencv2/core/utility.hpp>
#include 	<opencv2/opencv.hpp>
#include 	<opencv2/core.hpp>
#include 	<opencv2/aruco.hpp>

#include    "config.hpp"
#include    "types.hpp"

#include 	"VisionSystem.hpp"
#include 	"VisualizationOnImage.hpp"

enum DictSizes
{
    MARKERS_50 = 50,
    MARKERS_100 = 100,
    MARKERS_250 = 250,
    MARKERS_1000 = 1000
};

enum ArucoBitSizes
{
  ARUCO_4 = 4,
  ARUCO_5 = 5,
  ARUCO_6 = 6,
  ARUCO_7 = 7,
  ARUCO_ORIG = 1,
  APRILTAG_16 = 16,
  APRILTAG_25 = 25,
  APRILTAG_36H10 = 3610,
  APRILTAG_36H11 = 3611
};

const unsigned int dictArucoNumStep = 1000;

class AFiducialDetector
{
	public:
		explicit AFiducialDetector(std::vector<VisionSystem*>	&visionSysVector);
		~AFiducialDetector();
		virtual void detectFiducial() = 0;
		int   				detectedFiducialCount{};
		bool  				detectorStatus{};
		// вектор идентификаторов реперных маркеров
		// содержит в себе номера обнаруженных на изображении маркеров
		vec2i_t				idsList;
		vec1i_t				ids;
		//у каждогго словаря своя добавка
		vec1i_t				dictNumList;
		// вектор угловых точек
		// 	угловые точки являются вершинами реперного маркера (4 на каждый маркер т к он квадратный)
        vec3CvPoint2f_t		cornersList;
		vec2CvPoint2f_t		corners;

		vec1i_t 			dictsMarkersBitSize;
		vec1i_t 			dictSizes;
		
		std::vector<VisionSystem*>		visionSysVector;

	protected:
		void addIdsAndCorners(vec1i_t &ids, vec2CvPoint2f_t &corners, const unsigned int &index);

};


class ArucoDetector : public AFiducialDetector
{
	public:
		ArucoDetector(std::vector<VisionSystem*>   &visionSysVector, const std::string &configFile);
		~ArucoDetector() = default;
		void detectFiducial() override;
	private:
		void setDicts(const std::string &dictionaryConfigFile);
		// void updateParametrsFromConfig(const std::string &detectorParametrsConfigFile);
		// вектор словарей опорных маркеров
		// словарь - класс содержащий  в себе бинарные матрицы 
		// описывающие сигнатуру каждого опорного маркера
		// каждый маркер имеет свою уникальную структуру
		vec1arDict_t				  	dictionaryList_;
		// параметры алгоритма обнаружения опорных маркеров
		cvDetParams_t 					detectorParameters_;

};

#endif