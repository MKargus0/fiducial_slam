#ifndef 	VISUAL_NAVIGATION_INTERFACE_HPP
#define		VISUAL_NAVIGATION_INTERFACE_HPP

#include <chrono>

#include "types.hpp"
#include "VisionSystem.hpp"
#include "FiducialNavigationSystem.hpp"

/**
 * @brief класс интерфейс для работы с навигационными системами
 * 
 */
class VisualNavigation
{
    public:
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
        ~VisualNavigation() = default;
		/**
		 * @brief метод оценки положения 
		 * выполняет шаг измерений и оценки положения в СК карты
		 */
		void		estimatePosition();
		VectorXd_t&	getStateVector();
        
    private:
		void	calcSystemLoopTime();
		std::chrono::_V2::system_clock::time_point	currentSysTime_; // текущее время на шаге системы 
    	std::chrono::_V2::system_clock::time_point	lastSysTime_;    // время системы 
    	std::chrono::duration<double, std::milli>	timeStep_;	 // время цикла управления системы, шаг разница между текшим и предыдушим шагом
        std::vector<ANavigationSystem*>				navSystems_;
		double										loopTime_;   // system time step
		//статус навигационной системы (функцианирует / не функцианирует)
        int                			navStatus_;
		// Положение объекта в СК карты
		VectorXd_t					stateVector_;
		// ориентация объекта в СК карты
        Eigen::Vector3d				orientationAngles_;
		Eigen::Quaterniond			orientationQuat_;
		std::vector<VisionSystem*>	camVec_;
		// unsigned int 				flowIndex;
		// AopticalFlowNavigation*		flowNav;


};

#endif

