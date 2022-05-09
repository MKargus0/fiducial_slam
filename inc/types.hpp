#ifndef	 TYPES_HPP
#define	 TYPES_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     
#include <opencv2/aruco.hpp>

#include <Eigen/Geometry>

// #ifdef DEBUG
#define VISUALIZATION

typedef Eigen::Matrix<long double, Eigen::Dynamic, 1> 		VectorXd_t;
typedef cv::Ptr<cv::aruco::Dictionary> 						arDict_t;
typedef std::vector<cv::Vec3d>		   						vecCvVec3d_t;
typedef cv::Ptr<cv::aruco::DetectorParameters>				cvDetParams_t;
typedef std::vector<cv::Ptr<cv::aruco::Board>>				boardVect_t;
typedef cv::Ptr<cv::aruco::Board> 							cvBoard_t;
typedef std::vector<cv::Point3f>     						vec1CvPoint3f_t;
typedef std::vector<cv::Point2f>     						vec1CvPoint2f_t;
typedef	std::vector<std::vector<std::vector<cv::Point3f>>>	vec3CvPoint3f_t;
typedef std::vector<std::vector<std::vector<cv::Point2f>>>  vec3CvPoint2f_t;
typedef std::vector<std::vector<cv::Point2f>>  				vec2CvPoint2f_t;
typedef	std::vector<std::vector<cv::Point3f>>				vec2CvPoint3f_t;
typedef std::vector<int> 									vec1i_t;
typedef std::vector <std::vector<int>> 						vec2i_t;
typedef std::vector<double>									vec1d_t;
typedef std::vector<std::vector<long double>>				vec2ld_t;
typedef std::vector <cv::Ptr<cv::aruco::Dictionary>>  		vec1arDict_t;
typedef	std::vector<VectorXd_t>								vec1vecXd_t;
typedef	std::vector<std::vector<VectorXd_t>>				vec2vecXd_t;
// typedef std::vector <cv::InputArray>						vec1cvInpArr_t;
// typedef std::vector <cv::InputArrayOfArrays>				vec1cvInpArrOfArr_t;


#endif