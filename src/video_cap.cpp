#include "vision_systems.hpp"
#include "markerMap.hpp"
//using namespace std;

int main()
{
    cout << "test" << endl;
    


    // string config = "/home/argus/projects/visual_navigation/config/config.yaml";
    // fiducialMap map(0);


    // map.addFiducialToMap(0,0,0,0,0,0,1,0,4);


    string config = "/home/argus/projects/visual_navigation/config/config.yaml";
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