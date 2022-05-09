#ifndef		FIDUCIAL_NAVIGATION_FIDUCIAL_NAVIGATION_HPP
#define		FIDUCIAL_NAVIGATION_FIDUCIAL_NAVIGATION_HPP

#include 	<chrono>

#include 	"types.hpp"
#include	"config.hpp"

#include 	"FiducialDetector.hpp"
#include 	"ANavigationSystem.hpp"
#include	"FiducialMap.hpp"
#include    "CustomMultipleBoard.hpp"
#include	"CVMathOperations.hpp"
#include    "MathOperations.hpp"

class AfiducialNavigation : public ANavigationSystem
{
	public:
		AfiducialNavigation(unsigned int detectorType, std::vector<VisionSystem*>	visionSysVector,
							const std::string &detectorConfigFile);
		~AfiducialNavigation();

	protected:
		cv::Vec3d			tVec_;
		cv::Vec3d 			rVec_;
		// X Y Z roll pitch yaw
		VectorXd_t			posInMap_;
		AFiducialDetector	*fiducialDetector_;
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
		bool                  useExtrinsicGuess_ = false;
		double                axesSize_;
		CustomMultipleBoard*  board_;
		

};


class FiducialSlamNavigation : public FiducialBoardNavigation
{
	public:
		FiducialSlamNavigation(unsigned int detectorType, std::vector<VisionSystem*>	visionSysVector,
								const std::string &detectorConfigFile, const std::string &boardConfigFile);
		~FiducialSlamNavigation() = default;
		int estimateState() override;
		
	private:
		void 					updateMap();
		void 					updateIdsDetectedTime(const unsigned int &cameraId);
		void					deleteNotUpdatedMarkers();
		VectorXd_t				transformPoseToMap(cv::Vec3d& rvecMarker, cv::Vec3d& tvecMarker,
                                                     const unsigned int &cameraId);

		double					markerSize_;
		double					waitTimeBeforeMarkerAdd_;
		vec1i_t					unknownIds_;
		vec1i_t					unknownIdsIters_;
		vec1d_t					unknownMarkerTime;
		vec2ld_t				unknownMarkerPoseX;
		vec2ld_t				unknownMarkerPoseY;
		vec2ld_t				unknownMarkerPoseZ;
		vec2ld_t				unknownMarkerPoseRoll;
		vec2ld_t				unknownMarkerPosePitch;
		vec2ld_t				unknownMarkerPoseYaw;
		vecCvVec3d_t  			tVecs_;
		vecCvVec3d_t  			rVecs_;

};

#endif