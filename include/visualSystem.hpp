#ifndef 	VISUAL_SYSTEM_HPP
#define 	VISUAL_SYSTEM_HPP
#include 	<string>
#include 	<fstream>
#include 	<iostream>
#include 	<opencv2/opencv.hpp>
#include 	<yaml-cpp/yaml.h>
#include	<config.hpp>
#include 	<types.hpp>
#include 	<cvMathOperations.hpp>

/**
 * @brief клас хранит в себе изображение , и его параметры для обработки не используется 
 * 
 */
class VisionSystem
{	
	public:
		//объект обеспечивающий захват изображения их источника, служит для получения матрицы пикселей
    	cv::VideoCapture    inputVideo;
		// матрица пикселей получаемая из объекта захвата изображения
    	cv::Mat             image;
		// матрица фокусных расстояний и оптических центров камеры
    	cv::Mat             cameraMatrix;
		// матрица коэффициентов дисторсии камеры(модель )
    	cv::Mat             distCoeffs;
		//Положение и ориентаия камеры в осях связной системы координат объекта
		Eigen::Vector3d		cameraBodyPosition; // X Y Z (metrs)
		Eigen::Vector3d		cameraBodyOrientation; // roll pitch yaw (rad)
		/**
		 * @brief Construct a new Vision System object
		 * 
		 * @param video_source 
		 * @param width 
		 * @param height 
		 */
    	VisionSystem(const std::string &videoSource, const unsigned int &width, const unsigned int &height);
		/**
		 * @brief Construct a new Vision System object
		 * 
		 * @param videoSource 
		 * @param width 
		 * @param height 
		 */
		VisionSystem(const unsigned int &videoSource, const unsigned int &width, const unsigned int &height);
		/**
		 * @brief Construct a new Vision System object
		 * 
		 * @param configFile 
		 */
		VisionSystem(const std::string configFile);
		/**
		 * @brief Destroy the Vision System object
		 * закрывает источник видео на чтение, и закрывает все окна если они были открыты
		 */
    	~VisionSystem();
		/**
		 * @brief Set the Camera Position object
		 * 
		 * @param pose 
		 * @param orientation 
		 */
		void setCameraPosition(Eigen::Vector3d &pose, Eigen::Vector3d &orientation);
		VectorXd getCamPosition(cv::Vec3d &rvec, cv::Vec3d &tvec);
		void updateImage();
		void updateImage(cv::Mat &image);
	protected:

    	void   getCameraCalibration(const std::string &filename);

	private: 
		void	setVideoSize(const unsigned int &width, const unsigned int &height);
};


#endif