#include <fuducialNavigationSystem.hpp>


AfiducialNavigation::AfiducialNavigation(unsigned int detectorType, std::vector<*VisionSystem> visionSysVector)
	: AvisualNavigationSystem(visionSysVector)
{
	this->fiducialDetector = fiducialDetector;

	if ( detectorType == 0)
		fiducialDetector = new ArucoDetector(visionSysVector);
	else if ( detectorType == 1)
		fiducialDetector = new AprilTagDetector(visionSysVector);
	else
		std::cout << "Error fiducial type is not definded" << std::endl;

}

AfiducialNavigation::~AfiducialNavigation()
{
	delete fiducialDetector;
}


FiducialBoardNavigation::FiducialBoardNavigation(unsigned int detectorType, std::vector<*VisionSystem> visionSysVector)
	: AfiducialNavigation(detectorType, visionSysVector)
{

}

FiducialBoardNavigation::~FiducialBoardNavigation()
{

}


FiducialBoardNavigation::estimateState();
{

}

FiducialSlamNavigation::FiducialSlamNavigation(unsigned int detectorType, std::vector<*VisionSystem>	visionSysVector, fiducialMap *markerMap)
	:	FiducialBoardNavigation(detectorType,visionSysVector)
{
 	this->markerMap = markerMap;
}

FiducialSlamNavigation::~FiducialSlamNavigation()
{
	delete markerMap;
}