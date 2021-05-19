#pragma once
#include <visualSystem.hpp>
#include <types.hpp>

/**
 * @brief класс интерфейс для работы со вмеми навигационными системами
 * 
 */
class VisualNavigation
{
    public:

		//статус навигационной системы (функцианирует / не функцианирует)
        bool                	nav_status;
		// Положение обьекта в системе координат карты
        Eigen::Vector3d			position;
		// ориентация обьекта в системе координат карты
        Eigen::Vector3d			orientationAngles;
		Eigen::Quaterniond		orientationQuat;

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
        virtual void estimatePosition();
		void addCameraSource(unsigned int &source, unsigned int &width, unsigned int &height);
		void addCameraSource(const std::string &source, unsigned int &width, unsigned int &height);
		void addCameraSource(const std::string configfile);
        
    private:
        std::vector<*AnavigationSystem> navSystems;;
		

};

