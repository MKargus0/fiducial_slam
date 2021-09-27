#ifndef		FLOW_NAVIGATION_HPP
#define		FLOW_NAVIGATION_HPP
#include 	"types.hpp"
#include 	"visualNavigationSystem.hpp"
#include 	"visualizationOnImage.hpp"
#include	"config.hpp"

class AopticalFlowNavigation //: public AvisualNavigationSystem
{
	public:
		AopticalFlowNavigation(VisionSystem* visionSys, double fieldOfViewAngleWidth, double fieldOfViewAngleHeight, 
								int fieldWidth, int fieldHeight);
		
		~AopticalFlowNavigation();
		int calculateMovement(double omegaX, double omegaY, double range, double dt);
		virtual void calculateFlow() = 0;
		void	flowToRad();
		// int calculateMovement(double omegaX, double omegaY, double range, double dt);
		Eigen::Vector2d	flowVel;
		cv::Point2d		getFlowRad();
		cv::Point2d 	shiftRad;
		

	protected:
		cv::Mat 					currentFrame;
		cv::Mat 					previousFrame;
		cv::Mat 					currentFrame64f;
		cv::Mat 					previousFrame64f;
		cv::Mat 					hannWindow;
		cv::Point2d 				shift;
		std::vector<cv::Point2d>	shiftWithdist;
		std::vector<cv::Point2d>	shiftUndist;
		double						camFx;
		double						camFy;
		double						fieldOfViewAngleWidth;
		double						fieldOfViewAngleHeight;
		int							fieldWidth;
		int							fieldHeight;
		VisionSystem*				visionSys;
		Eigen::Vector2d				flowPose;


};


class PhaseCorr : public AopticalFlowNavigation
{
	public:
		PhaseCorr(VisionSystem* visionSys, double fieldOfViewAngleWidth, double fieldOfViewAngleHeight, 
								int fieldWidth, int fieldHeight);
		void calculateFlow() override;
	// private:
		

};

// class LucasCanadeFlow : public AopticalFlowNavigation
// {
// 	public:

// 	private:

// };

#endif