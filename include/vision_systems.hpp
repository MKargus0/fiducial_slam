#pragma once
#include <string> 
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/aruco.hpp>


#include <aruco_custom_tools.hpp>
#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
#include <yaml-cpp/yaml.h>
//#include <opencv2/core/persistence.hpp>


#define DVISUALIZATION

using namespace std; 

class vision_system
{
public:
    
    cv::VideoCapture    input_video;
    cv::Mat             image;
    cv::Mat             camera_matrix;
    cv::Mat             dist_coeffs;
    bool                use_video_input = true;

    vision_system(string video_source, int width, int height);
    ~vision_system();

    void                get_camera_calibration(string filename);
    cv::Vec3d           rvec_to_euler(cv::Vec3d rvec);
    Eigen::Quaterniond  euler_to_quat(cv::Vec3d angles);
    cv::Vec3d           vision_pose_transform(cv::Vec3d tvec, cv::Vec3d rvec);
    
    

};

class aruco_detector : public vision_system 
{
    public:
        
        //aruco_detector(int marker_bit_size =4,int dict_size=50,string video_source="0");
        

        int                                     marker_count;
        bool                                    is_nav;
        cv::Ptr<cv::aruco::DetectorParameters>  parameters;
        cv::Ptr<cv::aruco::Dictionary>          dictionary;
        std::vector<cv::Vec3d>                  rvecs;
        std::vector<cv::Vec3d>                  tvecs;
        cv::Vec3d                               tvec;
        cv::Vec3d                               rvec;
        cv::Vec3d                               euler_angles;
        // rvec вектор ориентации маркера относительно камеры
        // tvec - положение маркера относительно камеры
        std::vector<int>                        ids;
        std::vector<std::vector<cv::Point2f>>   corners;

        aruco_detector(int marker_bit_size,int dict_size,string video_source,int width,int height);
        ~aruco_detector();

        void            set_dict(int marker_bit_size,int dict_size); // устанавливаем словарь маркеров
        virtual void    marker_detect();
        //virtual void setup(string filename);
        virtual void    estimate_pose(){}
        


    private:

};

class navigate_single_marker : public aruco_detector
{
    public:

        navigate_single_marker(string filename, string video_source, int width, int height);
        void estimate_pose();
        void setup(string filename);
    private:

        double  marker_size;
        int     marker_id;

};

class navigate_board_marker : public aruco_detector
{
    public:

        navigate_board_marker(string filename, string video_source, int width, int height);
        void estimate_pose(int board_id);
        void setup(string filename);
        void marker_detect();
  
    private:

        int                                                 board_count;
        int                                                 marker_count;
        int                                                 board_detected_num;

        std::vector <cv::Ptr<cv::aruco::Board>>             board_list;
        cv::Ptr<cv::aruco::Board>                           board;
        std::vector<int>                                    dict_sizes;
        std::vector<int>                                    marker_bit_sizes;
        cv::Point3f                                         corn;
        std::vector <cv::Point3f>                           marker_corners;
        std::vector <std::vector<std::vector<cv::Point3f>>> rejected_candidates;
        std::vector <std::vector<int>>                      marker_ids;
        std::vector<float>                                  marker_params;
        std::vector<std::vector<cv::Point2f>>               markerCorners;
        std::vector <cv::Ptr<cv::aruco::Dictionary>>        dictionary_list;
        std::vector<std::vector<std::vector<cv::Point2f>>>  corners_list;

        void        add_board_config(string filename);
        void        set_marker_corners();
        cv::Point3f rotate_marker(cv::Point3f corn, float roll, float pitch, float yaw);
        
        #ifdef DVISUALIZATION
            cv::Mat image_copy;
        #endif

    

};

class navigate_slam_marker : public aruco_detector
{
    public:
        navigate_slam_marker();
        void estimate_pose();
        void setup(string filename);
        void addMarkerToMap();
    private:


};



class optical_flow_sys : public vision_system
{

};

class visual_navigation
{
    public:

        bool                nav_status;
        Eigen::Quaterniond  orientation;
        cv::Vec3d           translation;
        cv::Vec3d           vision_pose;
        cv::Vec3d           euler_angles;

        visual_navigation(string config_file);
        ~visual_navigation();
        void aruco_nav_do_step();
        void aruco_nav_do_step(cv::Mat &input_image);
        void flow_nav_do_step();
       
        
        

    private:
        
        int             width;
        int             height;    
        int             aruco_navigation_type; //single marker - 1  aruco board - 2 slam_algortm - 3     
        aruco_detector *ad;
        string          config;
        string          video_source;
        string          camera_calibration_path;
        string          aruco_config;
        bool            use_aruco;
        bool            use_flow;

        // void update_image();
        // void update_image(cv::Mat input_image);
        void aruco_setup();
        void single_marker_nav_setup();
        void marker_board_nav_setup();
        void marker_slam_nav_setup();
        void flow_setup();
        void set_navigation_data();
        

};
