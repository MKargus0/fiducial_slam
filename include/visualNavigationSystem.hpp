#ifndef 	VISUAL_NAVIGATION_SYSTEM_HPP
#define		VISUAL_NAVIGATION_SYSTEM_HPP
#include 	<navigationSystem.hpp>
#include	<visualSystem.hpp>
#include	<types.hpp>

class AvisualNavigationSystem :: public AnavigationSystem
{
	public:
		AvisualNavigationSystem(std::vector<*VisionSystem>	visionSysVector);
		~AvisualNavigationSystem();
	protected :
		std::vector<*VisionSystem>	visionSysVector;
		
};

#endif