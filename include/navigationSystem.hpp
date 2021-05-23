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
		virtual void estimateState() = 0;
};

#endif