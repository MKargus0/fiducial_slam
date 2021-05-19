#include <visualNavigationSystem.hpp>


AvisualNavigationSystem::AvisualNavigationSystem(std::vector<*VisionSystem>	visionSysVector)
{
	this->visionSysVector = visionSysVector;
}

AvisualNavigationSystem::~AvisualNavigationSystem()
{
	//очищаем память
	for (unsigned int i = 0; i < visionSysVector.size(); i++)
		delete visionSysVector[i];	
}