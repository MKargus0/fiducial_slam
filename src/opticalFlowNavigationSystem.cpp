#include "opticalFlowNavigationSystem.hpp"


AopticalFlowNavigation::AopticalFlowNavigation(VisionSystem* visionSys, double fieldOfViewAngleWidth, double fieldOfViewAngleHeight, 
								int fieldWidth, int fieldHeight)
{
	this->visionSys = visionSys;
	this->fieldWidth = fieldWidth;
	this->fieldHeight = fieldHeight;
	this->fieldOfViewAngleWidth = fieldOfViewAngleWidth;
	this->fieldOfViewAngleHeight = fieldOfViewAngleHeight;
	flowPose[0] = 0;
	flowPose[1] = 0;
	camFx = visionSys->cameraMatrix.at<double>(0,0);
	camFy = visionSys->cameraMatrix.at<double>(1,1);
	// cv::Point2d initShift;
	// shiftWithdist.push_back(initShift);
	shiftUndist.resize(1);
	
	
}

cv::Point2d		AopticalFlowNavigation::getFlowRad()
{
	return shiftRad;
}

void	AopticalFlowNavigation::flowToRad()
{
	shiftWithdist = {shift};
	// std::vector<cv::Point2d> shiftUndist(1);
	cv::undistortPoints(shiftWithdist, shiftUndist, visionSys->cameraMatrix, visionSys->distCoeffs, cv::noArray(), visionSys->cameraMatrix);
	shiftRad.x = atan2(shiftUndist[0].x, camFx);
 	shiftRad.y = atan2(shiftUndist[0].y, camFy);
	
}


AopticalFlowNavigation::~AopticalFlowNavigation()
{
	delete visionSys;
}

int AopticalFlowNavigation::calculateMovement(double omegaX, double omegaY, double range, double dt)
{
	calculateFlow();

	flowVel[0] = (range * fieldOfViewAngleWidth * shift.x) /  (dt * fieldWidth) 
	- (range * omegaY);

	flowVel[1] = (range * fieldOfViewAngleWidth * shift.y) /  (dt * fieldHeight) 
	- (range * omegaX);
}

void PhaseCorr::calculateFlow()
{
	cv::cvtColor(visionSys->image, currentFrame, cv::COLOR_RGB2GRAY);
	if(previousFrame.empty())
    {
        previousFrame = currentFrame.clone();
        cv::createHanningWindow(hannWindow, currentFrame.size(), CV_64F);
    }
	
	previousFrame.convertTo(previousFrame64f, CV_64F);
    currentFrame.convertTo(currentFrame64f, CV_64F);

	shift = cv::phaseCorrelate(previousFrame64f, currentFrame64f, hannWindow);

	#ifdef VISUALIZATION
		double radius = std::sqrt(shift.x*shift.x + shift.y*shift.y);
		if(radius > 5)
        {
            // draw a circle and line indicating the shift direction...
            cv::Point center(currentFrame.cols >> 1, currentFrame.rows >> 1);
            cv::circle(visionSys->image, center, (int)radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            cv::line(visionSys->image, center, cv::Point(center.x + (int)shift.x, center.y + (int)shift.y), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }
		VisualizationOnImage::showImage(visionSys->image, "flow_vizual");
	#endif
	
	previousFrame = currentFrame.clone();

}

PhaseCorr::PhaseCorr(VisionSystem* visionSys, double fieldOfViewAngleWidth, double fieldOfViewAngleHeight, 
								int fieldWidth, int fieldHeight): AopticalFlowNavigation(visionSys, fieldOfViewAngleWidth, fieldOfViewAngleHeight, 
								fieldWidth, fieldHeight)
{

}
