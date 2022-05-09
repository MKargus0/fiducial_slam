#ifndef FIDUCIAL_NAVIGATION_CUSTOM_FIDUCIAL_BOARD
#define	FIDUCIAL_NAVIGATION_CUSTOM_FIDUCIAL_BOARD

#include <fstream>
#include <iostream>

#include "types.hpp"
#include "CVMathOperations.hpp"
#include "config.hpp"


// class CastomDictionary
// {
// 	cv::Mat 	byteList;

// };

class CustomMultipleBoard
{
	public:
		CustomMultipleBoard(const unsigned int &dictionaryCount);
		// void addDataToBoard(vec3CvPoint3f_t &rejected_candidates, vec2i_t &ids);
		void addDataToBoard(cv::InputArrayOfArrays &objPoints, cv::InputArray &ids);
		void getBoardObjectAndImagePoints(cv::InputArray &detectedIds, cv::InputArrayOfArrays &detectedCorners);
		void addMarkerToMap(const unsigned int &markerId, const double &markerSize, double &boardX, double &boardY,
							const double &boardZ,const double &roll, const double &pitch,const double &yaw);
		void addMarkerToMap(const unsigned int &markerId, const double &markerSize, VectorXd_t &markerPose);
		void addMarkerToBoard(cv::InputArray &rejected_candidate, const unsigned int &markerId);
		cv::Mat					objPointsMat;
		cv::Mat 				imgPointsMat;
		vec2CvPoint3f_t 		objPoints;
		vec1i_t					ids;
		vec1i_t					unknownIds;
		vec2CvPoint2f_t			unKnownCorners;
		unsigned int 			dictionaryCount;
		void writeMapDataToFile();
		#ifdef	USE_PLOTTER
			void showPlotWithMarkers();
		#endif

	protected:
		void updateDataMap(const unsigned int &markerId,vec1CvPoint2f_t &markerCorners);

	private:
		#ifdef	USE_PLOTTER
			sciplot::Plot3D		plot;
			vec1d_t				dataX;
			vec1d_t				dataY;
			vec1d_t				dataZ;
			int					markerPlotIndex;
		#endif
		std::ofstream mapFile; // out file stream
			
};

#endif