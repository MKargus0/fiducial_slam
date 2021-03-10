#include "vision_systems.hpp"

vision_system::vision_system(string video_source,int width,int height)
{
    // if (!inputVideo.isOpened()){
    //     //error in opening the video input
    //     cerr << "Unable to open file!" << endl;
    // }
    // else {inputVideo.open(video_source);}
    //cout << "nnnnnaiosfnlkjnflkanflk\n";
    if(video_source != "none")
    {
        input_video.open(std::stoi(video_source));
        input_video.set(cv::CAP_PROP_FRAME_WIDTH,width);
        input_video.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    }
    else 
    {
        use_video_input = false;
        cout << "use video input from other source" << endl;
    }

}

cv::Vec3d vision_system::rvec_to_euler(cv::Vec3d rvec)
{
    cv::Mat rodr;
    cv::Rodrigues(rvec, rodr);

    //assert(isRotationMatrix(rodr));
    
    float sy = sqrt(rodr.at<double>(0,0) * rodr.at<double>(0,0) +  rodr.at<double>(1,0) * rodr.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(rodr.at<double>(2,1) , rodr.at<double>(2,2));
        y = atan2(-rodr.at<double>(2,0), sy);
        z = atan2(rodr.at<double>(1,0), rodr.at<double>(0,0));
    }
    else
    {
        x = atan2(-rodr.at<double>(1,2), rodr.at<double>(1,1));
        y = atan2(-rodr.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);

}

Eigen::Quaterniond vision_system::euler_to_quat(cv::Vec3d angles)
{
    Eigen::Quaterniond quat_yaw(std::cos(angles[2]/2),0,0,std::sin(angles[2]/2));
    Eigen::Quaterniond quat_pitch(std::cos(angles[1]/2),0,std::sin(angles[1]/2),0);
    Eigen::Quaterniond quat_roll(std::cos(angles[0]/2),std::sin(angles[0]/2),0,0);

    Eigen::Quaterniond final_rot_q = quat_yaw * quat_pitch * quat_roll;

    return final_rot_q;
}

cv::Vec3d vision_system::vision_pose_transform(cv::Vec3d tvec,cv::Vec3d rvec)
{
    cv::Mat result;
    cv::Mat rodr;
    cv::Rodrigues(rvec, rodr);
    //tvec = cv::Mat(tvec);
    result = -rodr.t() *  cv::Mat(tvec);
    return cv::Vec3d(result);
}

vision_system::~vision_system()
{
    input_video.release();

    #ifdef DVISUALIZATION
        cv::destroyAllWindows();
    #endif
}
//получаем калибровки камеры
void vision_system::get_camera_calibration(string filename)
{
    cv::FileStorage fs_cv(filename, cv::FileStorage::READ);
    fs_cv["camera_matrix"] >> camera_matrix;
    fs_cv["distortion_coefficients"] >> dist_coeffs;
    //YAML::Node fs = YAML::LoadFile(filename);
    //std::vector<float> camera_matrix = fs["camera_matrix"].as<std::vector<float>>();
    //dist_coeffs = fs["distortion_coefficients"].as<cv::Mat>();
    cout <<"gotovo\n";
    

}

aruco_detector::aruco_detector(int marker_bit_size,int dict_size,string video_source,int width,int height)
:vision_system(video_source,width,height)
{
    parameters = cv::aruco::DetectorParameters::create();
    set_dict(marker_bit_size,dict_size);
    is_nav = false;


}
aruco_detector::~aruco_detector()
{
    
}

// выбираем словарь маркеров(эталонная последовательность битов для идентификации маркера)
void aruco_detector::set_dict(int marker_bit_size,int dict_size)
{
    float check_ = true;
    switch(marker_bit_size)
    {
        case 4:
            switch (dict_size)
            {
                case 50:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
                    break;
                case 100:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
                    break;
                case 250:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
                    break;
                case 1000:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
                    break;
                default:
                    cout << "Error dict_size for bit_size" << to_string(marker_bit_size)
                     << " is not correct"<< endl;
                     check_ = false;
            }
            break;

        case 5:
            switch (dict_size)
            {
                case 50:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
                    break;
                case 100:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
                    break;
                case 250:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
                    break;
                case 1000:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
                    break;
                default:
                    cout << "Error dict_size for bit_size" << to_string(marker_bit_size)
                     << " is not correct"<< endl;
                    check_ = false;
            }
            break;

        
        case 6:
            switch (dict_size)
            {
                case 50:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
                    break;
                case 100:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
                    break;
                case 250:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
                    break;
                case 1000:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
                    break;
                default:
                    cout << "Error dict_size for bit_size" << to_string(marker_bit_size)
                     << " is not correct"<< endl;
                    check_ = false;
            }
            break;

        case 7:
            switch (dict_size)
            {
                case 50:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
                    break;
                case 100:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
                    break;
                case 250:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
                    break;
                case 1000:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
                    break;
                default:
                    cout << "Error dict_size for bit_size" << to_string(marker_bit_size)
                     << " is not correct"<< endl;
                    check_ = false;
            }
            break;

        case 1:
            switch (dict_size)
            {
                case 1:
                    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
                    break;
                default:
                    cout << "Check your dict_size input parametrs, can't choose dict"<< endl;
                    check_ = false;

            }
            break;
        default:
             cout << "Check your input parametrs, can't choose dict"<< endl;
             check_ = false;

        if(check_ == true)
        {   
            cout <<"dictionary :"<< to_string(marker_bit_size)
             <<" num: "<< to_string(dict_size) << endl;
        }
        
    }
}

void aruco_detector::marker_detect()
{
    

    if (!image.empty())
    {

        detectMarkers(image, dictionary, corners, ids, parameters);

        if (ids.size() > 0)
        {
            estimate_pose();
            is_nav = true;
            #ifdef DVISUALIZATION
                cv::aruco::drawDetectedMarkers(image,corners,ids);
            #endif
        }
        else
        {
            is_nav = false;
        }

        #ifdef DVISUALIZATION

            cv::namedWindow("out", cv::WINDOW_AUTOSIZE );
            cv::imshow("out", image);

        #endif
    }
}

navigate_single_marker::navigate_single_marker(string filename,string video_source,int width,int height)
    :aruco_detector(4,50,video_source,width,height)
{
    setup(filename);
}

navigate_board_marker::navigate_board_marker(string filename,string video_source,int width,int height)
: aruco_detector(4,50,video_source,width,height)
{
    setup(filename);
}

void navigate_single_marker::estimate_pose()
{
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    rvec = rvecs[0];
    tvec = tvecs[0];

    #ifdef DVISUALIZATION
                cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 1);
    #endif
    
    

    //         tvec = tvecs[i];
    // for(int i=0;i<ids.size();i++)
    // {   
    //     if(ids[i] == marker_id)
    //     {
    //         rvec = rvecs[i];
    //         tvec = tvecs[i];
    //         is_nav = true;
    //         #ifdef DVISUALIZATION
    //             cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[marker_id], tvecs[marker_id], 0.1);
    //         #endif
    //     }
    //     else{
    //         is_nav = false;
    //     }
            
    // }
    
}

void navigate_board_marker::marker_detect()
{
    if (!image.empty())
    {   
        #ifdef DVISUALIZATION
            image.copyTo(image_copy);
        #endif
        
        cv::Mat objPoints, imgPoints;// new

        board_detected_num = 0;
        for(int i=0;i<board_list.size();i++)
        {   
            //ids.clear();
            //corners.clear();
            detectMarkers(image,dictionary_list[i], corners, ids, parameters);
            if (ids.size() > 0)
            {   

                
                #ifdef DVISUALIZATION
                    cv::aruco::drawDetectedMarkers(image_copy,corners,ids);
                #endif

                //cout << board_list[i]->ids[0] << endl;
                get_board_object_and_image_points(board_list[i], corners, ids, objPoints, imgPoints);
                //cv::aruco::getBoardObjectAndImagePoints()
                if(objPoints.total() == 0) // 0 of the detected markers in board
                    //return 0;
                    {
                      //printf("text s cartimkoy\n");   
                    }
                else
                {
                    board_detected_num++;
                }
                // estimate_pose(i);
                
                
            }

            
        }
        if(board_detected_num > 0)
        {   
            //rvec = {0,0,0};
            //tvec = {0,0,0};
            bool useExtrinsicGuess = false;
            cv::solvePnP(objPoints, imgPoints, camera_matrix, dist_coeffs, rvec, tvec, useExtrinsicGuess);
            //(int)objPoints.total() / 4;
            // rvec = {0,0,0};
            // tvec = {0,0,0};
            // for(int i=0;i<board_detected_num;i++)
            // {
            //     rvec += rvecs[i];
            //     tvec += tvecs[i];
            // }
            // rvec /= board_detected_num;
            // tvec /= board_detected_num;
            is_nav = true;
            #ifdef DVISUALIZATION
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs, rvec, tvec, 0.4);
            #endif
        }
        else
        {
          is_nav = false;   
        }
        
        #ifdef DVISUALIZATION
            //cv::namedWindow("out", cv::WINDOW_NORMAL );
            //cv::resizeWindow("out", 1280, 960);
            cv::namedWindow("out", cv::WINDOW_AUTOSIZE );
            cv::imshow("out", image_copy);

            // cv::Mat imageUndistorted; // Will be the undistorted version of the above image.

            // cv::undistort(image, imageUndistorted, camera_matrix, dist_coeffs);

            // cv::namedWindow("out_dist", cv::WINDOW_AUTOSIZE );
            // cv::imshow("out_dist", imageUndistorted);

        #endif
        //cout<<"ok";
    }

}

void navigate_board_marker::estimate_pose(int board_id)
{
    //cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board_list[board_id], camera_matrix, dist_coeffs, rvecs[board_id], tvecs[board_id]);
    if(valid >0)
    {
        board_detected_num++;
    }
    //cout << "out";
}

void navigate_slam_marker::estimate_pose()
{

}

void navigate_single_marker::setup(string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    int marker_bit_size;
    int marker_dict;

    fs["marker_bit_size"] >> marker_bit_size;
    fs["marker_dict"] >> marker_dict;
    fs["marker_id"] >> marker_id;
    fs["marker_size"] >> marker_size;

    set_dict(marker_bit_size,marker_dict);


}

void navigate_board_marker::setup(string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["dict_count"] >> board_count;
    fs["dict_sizes"] >> dict_sizes;
    fs["marker_bit_sizes"] >> marker_bit_sizes;
    fs["marker_count"] >> marker_count;

    //размер массива идентефикаторов и полей опорных маркеров
    // устанавливаем в зависимости от используемых словарей 
    marker_ids.resize(dict_sizes.size());
    board_list.resize(dict_sizes.size());
    rejected_candidates.resize(dict_sizes.size());
    dictionary_list.resize(dict_sizes.size());
    rvecs.resize(dict_sizes.size());
    tvecs.resize(dict_sizes.size());
    for(int i = 0;i<marker_count;i++)
    {   
       fs["marker_"+to_string(i+1)] >>  marker_params;
       
       marker_corners.clear();
       corn.x = marker_params[2] - marker_params[8]/2;
       corn.y = marker_params[3] + marker_params[8]/2;
       corn.z = marker_params[4];
       corn = rotate_marker(corn,marker_params[5],marker_params[6],marker_params[7]);
       marker_corners.push_back(corn);
       corn.x = marker_params[2] + marker_params[8]/2;
       corn.y = marker_params[3] + marker_params[8]/2;
       corn.z = marker_params[4];
       corn = rotate_marker(corn,marker_params[5],marker_params[6],marker_params[7]);
       marker_corners.push_back(corn);
       corn.x = marker_params[2] + marker_params[8]/2;
       corn.y = marker_params[3] - marker_params[8]/2;
       corn.z = marker_params[4];
       corn = rotate_marker(corn,marker_params[5],marker_params[6],marker_params[7]);
       marker_corners.push_back(corn);
       corn.x = marker_params[2] - marker_params[8]/2;
       corn.y = marker_params[3] - marker_params[8]/2;
       corn.z = marker_params[4];
       corn = rotate_marker(corn,marker_params[5],marker_params[6],marker_params[7]);
       marker_corners.push_back(corn);
       
   
       //для добавления идентификатора я векторов угловых точек маркеров
       //проверяем параметры маркера на
       for(int j=0;j<dict_sizes.size();j++)
       {
        if(marker_params[0] == marker_bit_sizes[j])
        {
            marker_ids[j].push_back(marker_params[1]);
            rejected_candidates[j].push_back(marker_corners);
        }
       }
         
    }

    //создаем эталонные доски для каждого типа опорных маркеров в нашем поле
    cv::Mat board_img_all;
    for(int i = 0;i<board_list.size();i++)
    {   
        set_dict(marker_bit_sizes[i],dict_sizes[i]);
        dictionary_list[i] = dictionary;
        board_list[i] = cv::aruco::Board::create(rejected_candidates[i]
                                                ,dictionary,marker_ids[i]);

        std::vector<std::vector<cv::Point3f>> rc = rejected_candidates[i];
        std::vector <int> mi = marker_ids[i];
        //board = cv::makePtr<cv::aruco::Board>();
        board = cv::aruco::Board::create(rc,dictionary,mi);

        cout<< board->ids.size() << endl;
        // #ifdef DVISUALIZATION
        cv::Mat image_board;
        draw_planar_board(board_list[i],cv::Size(800, 800),image_board,50,1);
        //cv::aruco::drawPlanarBoard(board_list[i],cv::Size(800, 800),image_board,50,1);
        //     //cv::imshow("board_"+std::to_string(i), board_img);
        //     //cv::waitKey(0);
        if(!board_img_all.empty())
        {
          cv::Mat dst;
          //board_img_all.copyTo(board_img_all(cv::Rect(0,0,image_board.cols, image_board.rows)));
          cv::addWeighted(board_img_all,0.3,image_board,0.3,0,dst);
          board_img_all = dst;
          //cv2.addWeighted(img1,0.7,img2,0.3,0)
        }
        else
        {
            board_img_all = image_board;
        }
        
        // #endif
    }
    cv::imwrite("/home/argus/board_1.jpg",board_img_all);
    cout << "board has been generated "<<endl;
}

cv::Point3f navigate_board_marker::rotate_marker(cv::Point3f corn,float roll,float pitch,float yaw)
{
    Eigen::Quaternionf q_vector(0,corn.x-marker_params[2],corn.y-marker_params[3],corn.z-marker_params[4]); 
    Eigen::Quaternionf quat_yaw(std::cos(yaw/2),0,0,std::sin(yaw/2));
    Eigen::Quaternionf quat_pitch(std::cos(pitch/2),0,std::sin(pitch/2),0);
    Eigen::Quaternionf quat_roll(std::cos(roll/2),std::sin(roll/2),0,0);

    Eigen::Quaternionf final_rot_q = quat_yaw * quat_pitch * quat_roll;
    // cout <<std::to_string(final_rot_q.w()) <<endl;
    // cout <<std::to_string(final_rot_q.x()) <<endl;
    // cout <<std::to_string(final_rot_q.y()) <<endl;
    // cout <<std::to_string(final_rot_q.z()) <<endl;
    //Eigen::Quaterniond qua_pitch(1,1,1,1);
    Eigen::Quaternionf res = final_rot_q * q_vector * final_rot_q.inverse();

    cv::Point3f result = {res.x()+marker_params[2],res.y()+marker_params[3],res.z()+marker_params[4]};
    return result;
}

void navigate_slam_marker::setup(string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

}

void navigate_board_marker::add_board_config(string filename)
{
    
}



visual_navigation::visual_navigation(string config_file)
{
    config = config_file;

    cout << config_file << endl;
    //cv::FileStorage fs(config, cv::FileStorage::READ);
    //fs = new cv::FileStorage(config_file, cv::FileStorage::READ);
    //fs.open(config_file,cv::FileStorage::READ);
    //fs.open(config_file, cv::FileStorage::READ);
    YAML::Node fs = YAML::LoadFile(config_file);
    cout << "get data" << endl;

    use_aruco = fs["use_aruco"].as<bool>();
    aruco_navigation_type = fs["aruco_navigation_type"].as<int>();
    use_flow = fs["use_flow"].as<bool>();
    video_source = fs["video_source"].as<std::string>();
    width = fs["width"].as<int>();
    height = fs["height"].as<int>();
    cout << video_source << endl;
    //fs["use_aruco"] >> use_aruco;
    //fs["aruco_navigation_type"] >> aruco_navigation_type;
    //fs["use_flow"] >> use_flow;
    //fs["video_source"] >> video_source;  

    if(use_aruco == true)
    {   
       aruco_setup();
    }

    if(use_flow == true)
    {
        flow_setup();
    }

}

visual_navigation::~visual_navigation()
{
    ad->~aruco_detector();
    

}

void visual_navigation::aruco_setup()
{
    YAML::Node fs = YAML::LoadFile(config);
    camera_calibration_path = fs["camera_calibration_path"].as<std::string>();
    switch(aruco_navigation_type)
    {
        case 1:
            single_marker_nav_setup();
            break;

        case 2:
            marker_board_nav_setup();
            break;

        case 3:
            marker_slam_nav_setup();
            break;

        default:
            cout <<"check navigation type parametr, now is: "
            << to_string(aruco_navigation_type) << endl;
    }
}

void visual_navigation::flow_setup()
{

}

void visual_navigation::single_marker_nav_setup()
{   
    YAML::Node fs = YAML::LoadFile(config);;
    //fs["marker_config_path"] >> aruco_config;
    aruco_config = fs["marker_config_path"].as<std::string>();
    ad = new navigate_single_marker(aruco_config,video_source,width,height);
    ad->get_camera_calibration(camera_calibration_path); //получаем калибровку камеры
}

void visual_navigation::marker_board_nav_setup()
{
    YAML::Node fs = YAML::LoadFile(config);
    //fs["board_config_path"] >> aruco_config;
    aruco_config = fs["board_config_path"].as<std::string>();
    ad = new navigate_board_marker(aruco_config,video_source,width,height);
    ad->get_camera_calibration(camera_calibration_path); //получаем калибровку камеры

}

void visual_navigation::marker_slam_nav_setup()
{
    YAML::Node fs = YAML::LoadFile(config);
    //fs["marker_slam_config"] >> aruco_config;
    aruco_config = fs["marker_slam_config"].as<std::string>();
}


void visual_navigation::aruco_nav_do_step()
{   
    ad->input_video >> ad->image;
    ad->marker_detect();
    set_navigation_data();
}
void visual_navigation::aruco_nav_do_step(cv::Mat &input_image)
{   
    ad->image = input_image;
    ad->marker_detect();
    set_navigation_data();
    
}

void visual_navigation::set_navigation_data()
{
    //передаем классу навигационной системы параметры векторов положения и ориентации камеры
    nav_status = ad->is_nav;//наблюдаются ли ориентиры (да нет)
    ad->euler_angles = ad->rvec_to_euler(ad->rvec);// углы ориентации камеры относительно ориентиров
    //ad->euler_angles[0] += -3.145;
    //ad->euler_angles[2] += 180 * 3.145 / 180;
    euler_angles = ad->euler_angles;
    orientation = ad->euler_to_quat(ad->euler_angles);// кватернион ориентации камеры относительно ориентиров
    translation = ad->tvec;// положение ориентиров относительно визирной СК
    vision_pose = ad->vision_pose_transform(translation,ad->rvec);// положение каиеры в системе координат ориентиров
}
// void visual_navigation::update_image()
// {
//     ad->input_video >> ad->image;
// }

// void visual_navigation::update_image(cv::Mat input_image)
// {
//     ad->image = input_image;
// }
