#include "vision_systems.hpp"

//using namespace std;

int main()
{
    cout <<"test" << endl;
    int marker_bit_size = 4;
    int dict_size = 50;
    string video_source = "0";

    //aruco_detector markernav(marker_bit_size,dict_size,video_source);
    // markernav.input_video >> markernav.image;
    
    // markernav.marker_detect();
    // // cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    // // cv::imshow("Display Image", markernav.image);

    string config = "/home/argus/visual_navigation/config/config.yaml";
    visual_navigation vis_nav(config);

    while(true)
    {
        vis_nav.aruco_nav_do_step();
        char key = (char) cv::waitKey(2);
        if (key == 27)
            break;

    }
    // cv::Mat image;
    // image = cv::imread("/home/argus/catkin_ws/src/pathfinder/tools/chesboard_photo/ocam/test_img/test_1.jpg");
    // vis_nav.aruco_nav_do_step(image);
    // cv::waitKey(0);

    cout <<"end test" << endl;
    return 0;
}