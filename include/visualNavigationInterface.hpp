#pragma once
#include <visualSystem.hpp>
#include <types.hpp>
#include <fuducialNavigationSystem.hpp>
#include <chrono>

/**
 * @brief класс интерфейс для работы со вмеми навигационными системами
 * 
 */
class VisualNavigation
{
    public:

		//статус навигационной системы (функцианирует / не функцианирует)
        bool                		nav_status;
		// Положение обьекта в системе координат карты
        Eigen::Vector3d				position;
		// ориентация обьекта в системе координат карты
        Eigen::Vector3d				orientationAngles;
		Eigen::Quaterniond			orientationQuat;
		std::vector<VisionSystem*>	camVec;
		/**
		 * @brief Construct a new Visual Navigation object
		 * 
		 * @param configfile 
		 */
        VisualNavigation(const std::string &configfile);
		/**
		 * @brief Destroy the Visual Navigation object
		 * 
		 */
        ~VisualNavigation();
        // virtual void estimatePosition() = 0;
		void	estimatePosition();
	
        
    private:
		std::chrono::_V2::system_clock::time_point	currentSysTime; // текущее время на шаге системы 
    	std::chrono::_V2::system_clock::time_point	lastSysTime;    // время системы 
    	std::chrono::duration<double, std::milli>	timeStep;	 // время цикла управления системы, шаг разница между текшим и предыдушим шагом
        std::vector<AnavigationSystem*>				navSystems;
		double										loopTime;   // system time step
		void	calcSystemLoopTime();

		
		

};

