#ifndef	 TYPES_HPP
#define	 TYPES_HPP
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     
#include <opencv2/aruco.hpp>
#include <Eigen/Geometry>

typedef Eigen::Matrix<long double, Eigen::Dynamic, 1> 		VectorXd;
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
typedef std::vector <cv::Ptr<cv::aruco::Dictionary>>  		vec1arDict_t;
typedef	std::vector<VectorXd>								vec1vecXd_t;
// typedef std::vector <cv::InputArray>						vec1cvInpArr_t;
// typedef std::vector <cv::InputArrayOfArrays>				vec1cvInpArrOfArr_t;


#endif