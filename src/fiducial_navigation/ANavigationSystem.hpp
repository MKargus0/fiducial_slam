#ifndef FIDUCIAL_SLAM_FIDUCIAL_NAVIGATION_ANAVIGATION_SYSTEM_HPP
#define FIDUCIAL_SLAM_FIDUCIAL_NAVIGATION_ANAVIGATION_SYSTEM_HPP

#include "types.hpp"

/**
 * @brief base abstract class for navigation system
 * 
 */
class ANavigationSystem
{
	public:
		virtual int estimateState() = 0;
		void 		setLoopTime(double loopTime);
		double 		getLoopTime();
		VectorXd_t	getStateVector();

	protected:
		VectorXd_t 	stateVector_;
		double		loopTime_;
		
};

#endif