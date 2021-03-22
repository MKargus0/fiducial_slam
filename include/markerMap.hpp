#include <string> 
#include <Eigen/Geometry>
#include "mathOperations.hpp"
#include "iostream"

using namespace std;

class fiducialMap
{
    public:
        
        fiducialMap(int startmarker);
        ~fiducialMap();
        void loadMap(string filename);
        void writeMap(string filename);
        void addFiducialToMap(double X, double Y, double Z, double roll, double pitch, double yaw,
                                double marker_size,int markerId,
                                 int markerBitSize,const char *markerType = "ARUCO");
    private:
        struct fiducialMarker
        {
            double          markerX;
            double          markerY;
            double          markerZ;
            double          markerRoll;
            double          markerPitch;
            double          markerYaw;
            Eigen::Vector3d markerPoints[4];

        };
        int mapArraySize = 0;

        fiducialMarker *mapArray;


        

};