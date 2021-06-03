#ifndef CASTOM_FISUCIAL_BOARD
#define	CASTOM_FISUCIAL_BOARD
#include <types.hpp>
#include <cvMathOperations.hpp>
#include <config.hpp>

#ifdef VISUALIZATION
	#include <sciplot/sciplot.hpp>
#endif
class CastomDictionary
{
	cv::Mat 	byteList;

};

class CastomMultipleBoard
{
	public:

		cv::Mat					objPointsMat;
		cv::Mat 				imgPointsMat;
		vec2CvPoint3f_t 		objPoints;
		vec1i_t					ids;
		vec1i_t					unknownIds;
		vec2CvPoint2f_t			unKnownCorners;
		unsigned int 			dictionaryCount;
		
		
		CastomMultipleBoard(const unsigned int &dictionaryCount);
		// void addDataToBoard(vec3CvPoint3f_t &rejected_candidates, vec2i_t &ids);
		void addDataToBoard(cv::InputArrayOfArrays &objPoints, cv::InputArray &ids);
		void getBoardObjectAndImagePoints(cv::InputArray &detectedIds, cv::InputArrayOfArrays &detectedCorners);
		void addMarkerToMap(const unsigned int &markerId, const double &markerSize, double &boardX, double &boardY,
							const double &boardZ,const double &roll, const double &pitch,const double &yaw);
		void addMarkerToMap(const unsigned int &markerId, const double &markerSize, VectorXd &markerPose);
		void addMarkerToBoard(cv::InputArray &rejected_candidate, const unsigned int &markerId);
		#ifdef	VISUALIZATION
			void showPlotWithMarkers();
		#endif
		
	protected:
		void updateDataMap(const unsigned int &markerId,vec1CvPoint2f_t &markerCorners);
	
	private:
		#ifdef	VISUALIZATION
			sciplot::Plot3D		plot;
			vec1d_t				dataX;
			vec1d_t				dataY;
			vec1d_t				dataZ;
			int					markerPlotIndex;
		#endif

		
		
	
};

#endif