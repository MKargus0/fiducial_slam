#ifndef CV_TOOLS_ARUCO_CUSTOM_TOOLS 
#define CV_TOOLS_ARUCO_CUSTOM_TOOLS

#include <string> 

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/aruco.hpp>


void drawBoard(cv::aruco::Board *board, cv::Size outSize, cv::OutputArray img, int marginSize,
                     int borderBits);

void drawPlanarBoard(const cv::Ptr<cv::aruco::Board> &board, cv::Size outSize, cv::OutputArray img, int marginSize,
                     int borderBits);
					 
void getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints);


#endif

    