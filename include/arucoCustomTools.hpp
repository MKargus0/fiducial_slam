#include <string> 
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/aruco.hpp>


void draw_board(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits);

void draw_planar_board(const cv::Ptr<cv::aruco::Board> &_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits);
					 
void get_board_object_and_image_points(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints);

    