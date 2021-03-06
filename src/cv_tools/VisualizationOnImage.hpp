#ifndef CV_TOOLS_VISUALIZATION_ON_IMAGE_HPP
#define CV_TOOLS_VISUALIZATION_ON_IMAGE_HPP

#include "types.hpp"

#ifdef VISUALIZATION
/**
 * @brief класс осуществляющий вывод изображения на экран и визуализацию данных
 * 
 */
class VisualizationOnImage
{
	public:
		static void showImage(cv::Mat &image);
		static void showImage(cv::Mat &image, const std::string &windowName);
		static void drawMarkersAndAxes(cv::Mat &image, vec2CvPoint2f_t &corners, vec1i_t &ids,
									   cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cameraMatrix,
									   cv::Mat &distCoeffs, double &axesSize);
};
#endif

#endif