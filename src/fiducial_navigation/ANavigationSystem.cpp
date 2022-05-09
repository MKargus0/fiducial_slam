#include "ANavigationSystem.hpp"

void ANavigationSystem::setLoopTime(double loopTime)
{
	// получаем шаг системы по времени для работы алгоритма 
	this->loopTime_ = loopTime;
}

VectorXd_t	ANavigationSystem::getStateVector()
{
	return (stateVector_);
}

double ANavigationSystem::getLoopTime()
{
	return (loopTime_);
}