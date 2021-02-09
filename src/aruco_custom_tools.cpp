#include "aruco_custom_tools.hpp"

void draw_planar_board(const cv::Ptr<cv::aruco::Board> &_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits)
{
    draw_board(_board, outSize, _img, marginSize, borderBits);
}


void draw_board(cv::aruco::Board *_board, cv::Size outSize, cv::OutputArray _img, int marginSize,
                     int borderBits)
{
    //CV_Assert(!outSize.empty());
    CV_Assert(marginSize >= 0);
    // cv::Mat _img;
    _img.create(outSize, CV_8UC1);
    cv::Mat out = _img.getMat();
    out.setTo(cv::Scalar::all(255));
    out.adjustROI(-marginSize, -marginSize, -marginSize, -marginSize);
    
    // calculate max and min values in XY plane
    CV_Assert(_board->objPoints.size() > 0);
    float minX, maxX, minY, maxY;
    minX = maxX = _board->objPoints[0][0].x;
    minY = maxY = _board->objPoints[0][0].y;
    for(unsigned int i = 0; i < _board->objPoints.size(); i++) {
        for(int j = 0; j < 4; j++) {
                {
                    minX = cv::min(minX, _board->objPoints[i][j].x);
                    maxX = cv::max(maxX, _board->objPoints[i][j].x);
                    minY = cv::min(minY, _board->objPoints[i][j].y);
                    maxY = cv::max(maxY, _board->objPoints[i][j].y);
                }
        }
    }
    float sizeX = maxX - minX;
    float sizeY = maxY - minY;

    // proportion transformations
    float xReduction = sizeX / float(out.cols);
    float yReduction = sizeY / float(out.rows);

    if(xReduction > yReduction) {
        int nRows = int(sizeY / xReduction);
        int rowsMargins = (out.rows - nRows) / 2;
        out.adjustROI(-rowsMargins, -rowsMargins, 0, 0);
    } else {
        int nCols = int(sizeX / yReduction);
        int colsMargins = (out.cols - nCols) / 2;
        out.adjustROI(0, 0, -colsMargins, -colsMargins);
    }

    // now paint each marker
    cv::aruco::Dictionary &dictionary = *(_board->dictionary);
    cv::Mat marker;
    cv::Point2f outCorners[3];
    cv::Point2f inCorners[3];

    for(unsigned int m = 0; m < _board->objPoints.size(); m++) {
        // transform corners to markerZone coordinates
        for(int j = 0; j < 3; j++) {
            cv::Point2f pf = cv::Point2f(_board->objPoints[m][j].x, _board->objPoints[m][j].y);
            // move top left to 0, 0
            pf -= cv::Point2f(minX, minY);
            pf.x = pf.x / sizeX * float(out.cols);
            pf.y = (1.0f - pf.y / sizeY) * float(out.rows);
            outCorners[j] = pf;
        }

        // get marker
        cv::Size dst_sz(outCorners[2] - outCorners[0]); // assuming CCW order
        dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
        dictionary.drawMarker(_board->ids[m], dst_sz.width, marker, borderBits);

        if((outCorners[0].y == outCorners[1].y) && (outCorners[1].x == outCorners[2].x)) {
            // marker is aligned to image axes
            marker.copyTo(out(cv::Rect(outCorners[0], dst_sz)));
            continue;
        }

        // interpolate tiny marker to marker position in markerZone
        inCorners[0] = cv::Point2f(-0.5f, -0.5f);
        inCorners[1] = cv::Point2f(marker.cols - 0.5f, -0.5f);
        inCorners[2] = cv::Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

        // remove perspective
        cv::Mat transformation = cv::getAffineTransform(inCorners, outCorners);
        warpAffine(marker, out, transformation, out.size(),cv::INTER_NEAREST,
                         cv::BORDER_TRANSPARENT);
    }



}



// int estimate_pose_multiple_board(std::vector<std::vector<std::vector<cv::Point2f>>> corners_list, std::vector <std::vector <int>> marker_ids,
//                       const std::vector <cv::Ptr<cv::aruco::Board>> &board_list,
//                       cv::Mat _cameraMatrix, cv::Mat _distCoeffs, cv::Vec3d _rvec,
//                       cv::Vec3d _tvec, bool useExtrinsicGuess)
//  {

//     //CV_Assert(_corners.total() == _ids.total());

//     // get object and image points for the solvePnP function
//     cv::Mat objPoints, imgPoints;
//     for(int k=0;k<corners_list.size();k++)
//     {
//        //getBoardObjectAndImagePoints(board_list[i], corners_list[i], marker_ids[i], objPoints, imgPoints);
//         size_t nDetectedMarkers = marker_ids[k].total();
//         std::vector< cv::Point3f > objPnts;
//         objPnts.reserve(nDetectedMarkers);
        
//         std::vector< cv::Point2f > imgPnts;
//         imgPnts.reserve(nDetectedMarkers);

//         // look for detected markers that belong to the board and get their information
//         for(unsigned int i = 0; i < nDetectedMarkers; i++) {
//             int currentId = marker_ids[i].getMat().ptr< int >(0)[i];
//             for(unsigned int j = 0; j < board_list[i]->ids.size(); j++) {
//                 if(currentId == board_list[i]->ids[j]) {
//                     for(int p = 0; p < 4; p++) {
//                         objPnts.push_back(board_list[i]->objPoints[j][p]);
//                         imgPnts.push_back(detectedCorners.getMat(i).ptr< Point2f >(0)[p]);
//                     }
//                 }
//             }
//         }

//         // create output
//         Mat(objPnts).copyTo(objPoints);
//         Mat(imgPnts).copyTo(imgPoints);
//     }
    

//     CV_Assert(imgPoints.total() == objPoints.total());

//     if(objPoints.total() == 0) // 0 of the detected markers in board
//         return 0;

//     solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);

//     // divide by four since all the four corners are concatenated in the array for each marker
//     return (int)objPoints.total() / 4;
// }


void get_board_object_and_image_points(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints) {

    //CV_Assert(board->ids.size() == board->objPoints.size());
    //CV_Assert(detectedIds.total() == detectedCorners.total());

    size_t nDetectedMarkers = detectedIds.total();

    std::vector< cv::Point3f > objPnts;
    objPnts.reserve(nDetectedMarkers);

    std::vector< cv::Point2f > imgPnts;
    imgPnts.reserve(nDetectedMarkers);


    // look for detected markers that belong to the board and get their information
    for(unsigned int i = 0; i < nDetectedMarkers; i++) {
        int currentId = detectedIds.getMat().ptr< int >(0)[i];
        for(unsigned int j = 0; j < board->ids.size(); j++) {

            if(currentId == board->ids[j]) {
                for(int p = 0; p < 4; p++) {
                    objPnts.push_back(board->objPoints[j][p]);
                    imgPnts.push_back(detectedCorners.getMat(i).ptr< cv::Point2f >(0)[p]);
                }
            }
        }
    }

    // create output
    cv::Mat(objPnts).copyTo(objPoints);
    cv::Mat(imgPnts).copyTo(imgPoints);
}



// void draw_marker(int id, int sidePixels, cv::OutputArray _img, int borderBits) const {

//     CV_Assert(sidePixels >= (markerSize + 2*borderBits));
//     CV_Assert(id < bytesList.rows);
//     CV_Assert(borderBits > 0);

//     _img.create(sidePixels, sidePixels, CV_8UC1);

//     // create small marker with 1 pixel per bin
//     cv::Mat tinyMarker(markerSize + 2 * borderBits, markerSize + 2 * borderBits, CV_8UC1,
//                    cv::Scalar::all(0));
//     cv::Mat innerRegion = tinyMarker.rowRange(borderBits, tinyMarker.rows - borderBits)
//                           .colRange(borderBits, tinyMarker.cols - borderBits);
//     // put inner bits
//     cv::Mat bits = 255 * cv::aruco::getBitsFromByteList(bytesList.rowRange(id, id + 1), markerSize);
//     CV_Assert(innerRegion.total() == bits.total());
//     bits.copyTo(innerRegion);

//     // resize tiny marker to output size
//     cv::resize(tinyMarker, _img.getMat(), _img.getMat().size(), 0, 0, cv::INTER_NEAREST);
// }