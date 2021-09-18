#include <visualizationOnImage.hpp>


void VisualizationOnImage::showImage(cv::Mat &image)
{
	cv::namedWindow("out", cv::WINDOW_NORMAL);
	cv::imshow("out", image);
	#ifndef ENABLE_QUIT_FROM_WINDOW
		cv::waitKey(1);
	#endif
}

void VisualizationOnImage::showImage(cv::Mat &image, const std::string &windowName)
{
	cv::namedWindow(windowName, cv::WINDOW_NORMAL);
	cv::imshow(windowName, image);
	#ifndef ENABLE_QUIT_FROM_WINDOW
		cv::waitKey(1);
	#endif
}

void VisualizationOnImage::drawMarkersAndAxes(cv::Mat &image, vec2CvPoint2f_t &corners, vec1i_t &ids, cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, double &axesSize)
{
	cv::aruco::drawDetectedMarkers(image, corners, ids);
	cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, axesSize);
	// showImage(image);
}