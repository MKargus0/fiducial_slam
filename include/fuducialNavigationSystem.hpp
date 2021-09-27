#ifndef		FIDUCIAL_NAVIGATION_HPP
#define		FIDUCIAL_NAVIGATION_HPP
#include 	"types.hpp"
#include 	"fiducialDetector.hpp"
#include 	"visualNavigationSystem.hpp"
#include	"fiducialMap.hpp"
#include	"config.hpp"
#include	"customBoard.hpp"
#include	"cvMathOperations.hpp"
#include 	"mathOperations.hpp"
#include 	<chrono>

class AfiducialNavigation : public AvisualNavigationSystem
{
	public:
		AfiducialNavigation(unsigned int detectorType, std::vector<VisionSystem*>	visionSysVector, const std::string &detectorConfigFile);
		~AfiducialNavigation();
		cv::Vec3d			tvec;
		cv::Vec3d 			rvec;
		// X
		// Y
		// Z
		// roll
		// pitch
		// yaw
		VectorXd			posInMap;

	protected:
		AfiducialDetector	*fiducialDetector;
};



class FiducialBoardNavigation : public AfiducialNavigation
{
	public:
		FiducialBoardNavigation(unsigned int detectorType, std::vector<VisionSystem*>	visionSysVector, const std::string &detectorConfigFile, const std::string &boardConfigFile);
		~FiducialBoardNavigation();
		int estimateState() override;

	protected:
		// при значении параметра Истина при решении задачи фотограмметрии учитывается предыдущее положение особых точек
		// в таком режиме положение не может поменяться резко что в нашем случае неприемлимо т к если маркеры на какое то время 
		// будут потеряны из поля зрения системы положение будет возвращаться к истинному состоянию за некоторое время 
		bool useExtrinsicGuess = false;
		double axesSize;
		CastomMultipleBoard* Board;
		

};


class FiducialSlamNavigation : public FiducialBoardNavigation
{
	public:
		FiducialSlamNavigation(unsigned int detectorType, std::vector<VisionSystem*>	visionSysVector, const std::string &detectorConfigFile, const std::string &boardConfigFile);
		// ~FiducialSlamNavigation();
		int estimateState() override;
		
	private:
		// fiducialMap		*markerMap;
		std::vector<cv::Vec3d>  tvecs;
		std::vector<cv::Vec3d>  rvecs;
		
		double					markerSize;
		double					waitTimeBeforeMarkerAdd;
		vec1i_t					unknownIds;
		vec1i_t					unknownIdsIters;
		vec1d_t					unknownMarkerTime;
		// vec1vecXd_t				unknownMarkerPose;
		vec2ld_t					unknownMarkerPoseX;
		vec2ld_t					unknownMarkerPoseY;
		vec2ld_t					unknownMarkerPoseZ;
		vec2ld_t					unknownMarkerPoseRoll;
		vec2ld_t					unknownMarkerPosePitch;
		vec2ld_t					unknownMarkerPoseYaw;
		void 					updateMap();
		void 					updateIdsDetectedTime(const unsigned int &cameraId);
		void					deliteNotUpdatedMarkers();
		VectorXd				transformPoseToMap(cv::Vec3d &rvecMarker, cv::Vec3d &tvecMarker, const unsigned int &cameraId);

};

#endif