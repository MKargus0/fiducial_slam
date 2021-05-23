#include <visualSystem.hpp>

VisionSystem::VisionSystem(const std::string &videoSource, const unsigned int &width, const unsigned int &height)
{
	inputVideo.open(videoSource);
	setVideoSize(width, height);

	//debug
	std::cout << "itit image source 1conf" << std::endl;	
}

VisionSystem::VisionSystem(const unsigned int &videoSource, const unsigned int &width, const unsigned int &height)
{
	inputVideo.open(videoSource);
	setVideoSize(width, height);

	std::cout << "itit image source 2conf" << std::endl;
}

VisionSystem::VisionSystem(const std::string configFile)
{
	getCameraCalibration(configFile);

	// unsigned int sourceType;
	// fs_cv["source_type"] >> sourceType;
	// if (sourceType == 0)
	// {
	// 	fs_cv["video_source"] >> sourceType;
	// }

	cv::FileStorage fs_cv(configFile, cv::FileStorage::READ);
	//получение источника из конфиг файла нужно проработать
	int source;
	fs_cv["video_source"] >> source;
	//утечки памяти возникают если не выставить бэкэнд
	// бэкэнд выставляется для того чтобы выбрать способ ролучения изображения
	// автоматический бэкэнд выбирает gstreamer а с ним утечки, такой не ставим cv::CAP_GSTREAMER
	int Backend = cv::CAP_V4L2;
	inputVideo.open(source,Backend);
	int width;
	int height;
	fs_cv["image_width"] >> width;
	fs_cv["image_height"] >> height;
	setVideoSize(width, height);

	fs_cv["camera_X"] >> cameraPosition[0];
	fs_cv["camera_Y"] >> cameraPosition[1];
	fs_cv["camera_Z"] >> cameraPosition[2];
	fs_cv["camera_roll"] >> cameraOrientation[0];
	fs_cv["camera_pitch"] >> cameraOrientation[1];
	fs_cv["camera_yaw"] >> cameraOrientation[2];

	//debug
	std::cout << "itit image source 3conf" << std::endl;

}

VisionSystem::~VisionSystem()
{
	inputVideo.release();
    #ifdef VISUALIZATION
        cv::destroyAllWindows();
    #endif
}

void VisionSystem::setCameraPosition(Eigen::Vector3d &pose, Eigen::Vector3d &orientation)
{
	cameraPosition = pose;
	cameraOrientation = orientation;
}

void VisionSystem::getCameraCalibration(const std::string &filename)
{
	cv::FileStorage fs_cv(filename, cv::FileStorage::READ);
	fs_cv["camera_matrix"] >> cameraMatrix;
    fs_cv["distortion_coefficients"] >> distCoeffs;
}


void VisionSystem::setVideoSize(const unsigned int &width, const unsigned int &height)
{
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,width);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,height);
}

VectorXd VisionSystem::getCamPosition(cv::Vec3d &rvec, cv::Vec3d &tvec)
{
	VectorXd result;
	result.resize(6);
	cv::Vec3d pose = CvMathOperations::visionPoseTransformToLocalFrame(tvec, rvec);
	cv::Vec3d angles = CvMathOperations::rvecToEuler(rvec);

	result[0] = pose[0];
	result[1] = pose[1];
	result[2] = pose[2];
	result[3] = angles[0];
	result[4] = angles[1];
	result[5] = angles[2];
	return result;

}