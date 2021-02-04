#include <string> 
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/aruco.hpp>


void draw_board(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits);

void draw_planar_board(const cv::Ptr<cv::aruco::Board> &_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits);

// int estimate_pose_multiple_board(std::vector<std::vector<std::vector<cv::Point2f>>> corners_list, std::vector <std::vector <int>> marker_ids,
//                       const std::vector <cv::Ptr<cv::aruco::Board>> &board_list,
//                       cv::Mat _cameraMatrix, cv::Mat _distCoeffs, cv::Vec3d _rvec,
//                       cv::Vec3d _tvec, bool useExtrinsicGuess);

void get_board_object_and_image_points(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints);

    