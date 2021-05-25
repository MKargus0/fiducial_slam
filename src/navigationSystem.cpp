#include <navigationSystem.hpp>

void AnavigationSystem::setLoopTime(double &loopTime)
{
	// получаем шаг системы по времени для работы алгоритма 
	this->loopTime = loopTime;
}

double AnavigationSystem::getLoopTime()
{
	return loopTime;
}