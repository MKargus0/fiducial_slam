#include "ArucoCustomTools.hpp"

void drawPlanarBoard(const cv::Ptr<cv::aruco::Board> &board, cv::Size outSize, cv::OutputArray img, int marginSize,
                     int borderBits)
{
    drawBoard(board, outSize, img, marginSize, borderBits);
}


void drawBoard(cv::aruco::Board *board, cv::Size outSize, cv::OutputArray img, int marginSize,
                     int borderBits)
{
    //CV_Assert(!outSize.empty());
    CV_Assert(marginSize >= 0);
    // cv::Mat _img;
    img.create(outSize, CV_8UC1);
    cv::Mat out = img.getMat();
    out.setTo(cv::Scalar::all(255));
    out.adjustROI(-marginSize, -marginSize, -marginSize, -marginSize);
    
    // calculate max and min values in XY plane
    CV_Assert(board->objPoints.size() > 0);
    float minX, maxX, minY, maxY;
    minX = maxX = board->objPoints[0][0].x;
    minY = maxY = board->objPoints[0][0].y;
    for(unsigned int i = 0; i < board->objPoints.size(); i++) {
        for(int j = 0; j < 4; j++) {
                {
                    minX = cv::min(minX, board->objPoints[i][j].x);
                    maxX = cv::max(maxX, board->objPoints[i][j].x);
                    minY = cv::min(minY, board->objPoints[i][j].y);
                    maxY = cv::max(maxY, board->objPoints[i][j].y);
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
    cv::aruco::Dictionary &dictionary = *(board->dictionary);
    cv::Mat marker;
    cv::Point2f outCorners[3];
    cv::Point2f inCorners[3];

    for(unsigned int m = 0; m < board->objPoints.size(); m++) {
        // transform corners to markerZone coordinates
        for(int j = 0; j < 3; j++) {
            cv::Point2f pf = cv::Point2f(board->objPoints[m][j].x, board->objPoints[m][j].y);
            // move top left to 0, 0
            pf -= cv::Point2f(minX, minY);
            pf.x = pf.x / sizeX * float(out.cols);
            pf.y = (1.0f - pf.y / sizeY) * float(out.rows);
            outCorners[j] = pf;
        }

        // get marker
        cv::Size dst_sz(outCorners[2] - outCorners[0]); // assuming CCW order
        dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
        dictionary.drawMarker(board->ids[m], dst_sz.width, marker, borderBits);

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

void getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> &board, cv::InputArrayOfArrays detectedCorners,
    cv::InputArray detectedIds, cv::OutputArray objPoints, cv::OutputArray imgPoints) {

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