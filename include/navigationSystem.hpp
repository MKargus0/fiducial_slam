#ifndef NAVIGATION_SYSTEM_HPP
#define NAVIGATION_SYSTEM_HPP
#include <types.hpp>

/**
 * @brief 
 * 
 */
class AnavigationSystem
{
	public:
		VectorXd stateVector;
		virtual int estimateState() = 0;
		void setLoopTime(double &loopTime);
		double getLoopTime();
	private:
		double		loopTime;
		
		
};

#endif