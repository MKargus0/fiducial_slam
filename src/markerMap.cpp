#include "markerMap.hpp"


using namespace std;

fiducialMap::fiducialMap(int startmarker)
{
    cout << "hey i am marker map constructor" << endl;

}
fiducialMap::~fiducialMap()
{
    cout << "hey i am marker map destructor" << endl;

}
void fiducialMap::loadMap(string filename)
{

}

void fiducialMap::writeMap(string filename)
{

}


void fiducialMap::addFiducialToMap(double X, double Y, double Z, double roll, double pitch,
                                     double yaw, double marker_size, int markerId, int markerBitSize
                                     ,const char *markerType)
{
    cout << "hey i am marker map" << endl;
    fiducialMarker *temp = new fiducialMarker[mapArraySize+1];
    for (unsigned int i = 0; i < mapArraySize; i++)
    {
        temp[i] = mapArray[i];
    }

    mapArraySize++;
    fiducialMarker fiducial;
    fiducial.markerX = X;
    fiducial.markerY = Y;
    fiducial.markerZ = Z;
    fiducial.markerRoll = roll;
    fiducial.markerPitch = pitch;
    fiducial.markerYaw = yaw;

            // markerPoints[0] = rotateVector();
    //разворачиваем маркеры на нужные углы для этого разворачиваем точки в системе координат маркера
    fiducial.markerPoints[0] = rotateVector(-marker_size / 2, marker_size / 2, 0,roll,pitch,yaw);
    fiducial.markerPoints[1] = rotateVector(marker_size / 2, marker_size  / 2, 0,roll,pitch,yaw);
    fiducial.markerPoints[2] = rotateVector(marker_size / 2, -marker_size / 2, 0,roll,pitch,yaw);
    fiducial.markerPoints[3] = rotateVector(-marker_size / 2, -marker_size / 2, 0,roll,pitch,yaw);

    //переходим из системы координат маркера в систему координат карты
    for (int i = 0; i < 4; i++)
    {
        fiducial.markerPoints[i][0] += X;
        fiducial.markerPoints[i][1] += Y;
        fiducial.markerPoints[i][2] += Z;
    }
    
    temp[mapArraySize] = fiducial;
    delete[] mapArray;
    mapArray = temp;
    delete[] temp;

}                                      