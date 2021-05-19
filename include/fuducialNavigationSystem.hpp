#ifndef		FIDUCIAL_NAVIGATION_HPP
#define		FIDUCIAL_NAVIGATION_HPP
#include 	<types.hpp>
#include 	<fiducialDetector.hpp>
#include 	<visualNavigationSystem.hpp>
#include	<fiducialMap.hpp>
#include	<config.hpp>


class AfiducialNavigation : public AvisualNavigationSystem
{
	public:

		AfiducialNavigation(unsigned int detectorType, std::vector<*VisionSystem>	visionSysVector);
		~AfiducialNavigation();
		cv::Vec3d			tvec;
		cv::Vec3d 			rvec;
		Eigen::Vector3d		position;
		Eigen::Vector3d		orientation;

	protected:
		AfiducialDetector	*fiducialDetector;
};



class FiducialBoardNavigation : public AfiducialNavigation
{
	public:
		FiducialBoardNavigation(unsigned int detectorType, std::vector<*VisionSystem>	visionSysVector)
		~FiducialBoardNavigation();
		void estimateState() override;



};


class FiducialSlamNavigation : public FiducialBoardNavigation
{
	public:
		FiducialSlamNavigation(unsigned int detectorType, std::vector<*VisionSystem>	visionSysVector, fiducialMap	*markerMap);
		~FiducialSlamNavigation();
		void estimateState() override;
	private:
		fiducialMap		*markerMap;

};

#endif