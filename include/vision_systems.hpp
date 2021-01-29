#include <string> 
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/aruco.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
#include <yaml-cpp/yaml.h>


#define VISUALZATION

using namespace std; 

class vision_system
{
public:
    vision_system(string video_source);
    void get_camera_calibration(string filename);
    cv::VideoCapture input_video;
    cv::Mat image;
    ~vision_system();
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    bool use_video_input = true;
   
    

};

class aruco_detector : public vision_system 
{
    public:
        int marker_count;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        std::vector<cv::Vec3d> rvecs, tvecs; 
        // rvec вектор ориентации маркера относительно камеры
        // tvec - положение маркера относительно камеры

        aruco_detector(int marker_bit_size,int dict_size,string video_source);
        //aruco_detector(int marker_bit_size =4,int dict_size=50,string video_source="0");
        ~aruco_detector();
        void set_dict(int marker_bit_size,int dict_size); // устанавливаем словарь маркеров
        virtual void marker_detect();
        //virtual void setup(string filename);
        virtual void estimate_pose(){}
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;

    private:

};

class navigate_single_marker : public aruco_detector
{
    public:
        navigate_single_marker(string filename,string video_source);
        void estimate_pose();
        void setup(string filename);
    private:
        double marker_size;
        int marker_id;

};

class navigate_board_marker : public aruco_detector
{
    public:
        navigate_board_marker(string filename,string video_source);
        void estimate_pose(int board_id);
        void setup(string filename);
        void marker_detect();

    private:
        int board_count;
        int marker_count;
        void add_board_config(string filename);
        void set_marker_corners();
        cv::Point3f rotate_marker(cv::Point3f corn,float roll,float pitch,float yaw);
        std::vector <cv::Ptr<cv::aruco::Board>> board_list;
        cv::Ptr<cv::aruco::Board>  board;
        std::vector<int> dict_sizes;
        std::vector<int> marker_bit_sizes;
        cv::Point3f corn;
        std::vector < cv::Point3f > marker_corners;
        std::vector <std::vector <std::vector < cv::Point3f > > > rejected_candidates;
        std::vector <std::vector <int>> marker_ids;
        std::vector<float> marker_params;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector <cv::Ptr<cv::aruco::Dictionary>> dictionary_list;
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        int board_detected_num;
        
        #ifdef VISUALZATION
            cv::Mat  image_copy;
        #endif

    

};

class navigate_slam_marker : public aruco_detector
{
    public:
        navigate_slam_marker();
        void estimate_pose();
        void setup(string filename);

};



class optical_flow_sys : public vision_system
{

};

class visual_navigation
{
    public:
        visual_navigation(string config_file);
        
        void aruco_nav_do_step();
        void aruco_nav_do_step(cv::Mat &input_image);
        void flow_nav_do_step();

    private:
        
        void aruco_setup();
        void single_marker_nav_setup();
        void marker_board_nav_setup();
        void marker_slam_nav_setup();
        void flow_setup();

        // void update_image();
        // void update_image(cv::Mat input_image);
        aruco_detector *ad;
        string config;
        string video_source;
        string camera_calibration_path;
        bool use_aruco;
        int aruco_navigation_type; //single marker - 1  aruco board - 2 slam_algortm - 3 
        string aruco_config;

        bool use_flow;
        

};
