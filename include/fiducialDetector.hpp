#ifndef 	FIDUCIAL_DETECTOR_HPP
#define 	FIDUCIAL_DETECTOR_HPP
#include 	<types.hpp>
#include 	<visualSystem.hpp>
#include	<config.hpp>
#include 	<opencv2/core/utility.hpp>
#include 	<opencv2/opencv.hpp>
#include 	<opencv2/core.hpp>    
#include 	<opencv2/aruco.hpp>
#include 	<visualizationOnImage.hpp>

class AfiducialDetector 
{
	public:
		int   				detectedFiducialCount;
		bool  				detectorStatus;
		// вектор идентификаторов реперных маркеров
		// содержит в себе номера обнаруженных на изображении маркеров
		vec2i_t				idsList;
		// вектор угловых точек
		// 	угловые точки являютя вершинами реперного маркера (4 на каждый маркер т к он квадратный)
        vec3CvPoint2f_t		cornersList;
		
		AfiducialDetector(std::vector<VisionSystem*>	&visionSysVector);
		~AfiducialDetector();
		virtual void detectFidusial() = 0;
	protected:
		std::vector<VisionSystem*>		visionSysVector;

};


class ArucoDetector : public AfiducialDetector
{
	public:
		ArucoDetector(std::vector<VisionSystem*>   &visionSysVector, const std::string &configFile);
		~ArucoDetector();
		void detectFidusial() override;
	private:
		// вектор словарей опорных маркеров
		// словарь - класс содержащий  в себе бинарные матрицы 
		// описывающие сигнатуру каждого опорного маркера
		// каждый маркер имеет свою уникальную структуру
		vec1arDict_t				  	dictionaryList;
		// параметры алгоритма обнаружения опорных маркеров
		cvDetParams_t 					detectorParametrs;


		void updateParametrsFromCongig(const std::string &detectorParametrsConfigFile);
		void setDicts(const std::string &dictionaryConfigFile);


};



class AprilTagDetector : public AfiducialDetector
{
	public:
		AprilTagDetector(std::vector<VisionSystem*>	&visionSysVector);
		~AprilTagDetector();
		void detectFidusial() override;
};





#endif