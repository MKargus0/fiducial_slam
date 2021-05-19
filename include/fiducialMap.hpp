#ifndef MARKER_MAP_HPP
#define MARKER_MAP_HPP
#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Geometry>
#include <mathOperations.hpp>



using namespace std;

class fiducialMap
{
    public:
        
        fiducialMap(int startmarker);
        ~fiducialMap();
        void loadMap(string filename);
        void writeMap(string filename);
        void showMap();
        void addFiducialToMap(Eigen::Vector3d &position, Eigen::Vector3d &orientation,
                                double &markerSize,int &markerId,
                                 int markerBitSize,const char *markerType = "ARUCO");
		void checkNewMarker();
    private:
        struct fiducialMarker
        {
			// fiducialMarker()
			// {
			// 	Eigen::Vector3d.
			// 	for(unsigned int i = 0; i < 4; i++)
			// 	{
			// 		markerPoints[i][0] = 0;
			// 		markerPoints[i][1] = 0;
			// 		markerPoints[i][2] = 0;
			// 	}
			// }
            Eigen::Vector3d position;
            Eigen::Vector3d orientation;
            Eigen::Vector3d markerPoints[4];

        };
        std::vector<fiducialMarker> mapArray;    

};


#endif