#ifndef CALIBSTOPSIGN_H
#define CALIBSTOPSIGN_H

#include "gc_types.h"
#include <opencv2/core.hpp>
#include "stopsignsearch.h"

// TODO -- add doxygen comments KWC
namespace gc
{

class StopSignCandidate
{
public:
    StopSignCandidate() :
        area( -1.0 ),
        elongation( -1.0 )
    {}

    StopSignCandidate( std::vector< cv::Point > &vecPts, double contourArea, double contourElongation ) :
        contour( vecPts ),
        area( contourArea ),
        elongation( contourElongation )
    {}

    void clear()
    {
        contour.clear();
        area = -1.0;
        elongation = -1.0;
    }

    std::vector< cv::Point > contour;
    double area;
    double elongation;
};
class StopSignLine
{
public:
    StopSignLine() :
        pt1( cv::Point2d( -1.0, -1.0 ) ),
        pt2( cv::Point2d( -1.0, -1.0 ) )
    {}

    StopSignLine( const cv::Point2d point1, const cv::Point2d point2 ) :
        pt1( point1 ),
        pt2( point2 )
    {}

    void clear()
    {
        pt1 = cv::Point2d( -1.0, -1.0 );
        pt2 = cv::Point2d( -1.0, -1.0 );
    }

    cv::Point2d pt1;
    cv::Point2d pt2;
};

class OctagonLines
{
public:
    OctagonLines() {}

    void clear()
    {
        top.clear();
        topRight.clear();
        right.clear();
        botRight.clear();
        bot.clear();
        botLeft.clear();
        left.clear();
        topLeft.clear();
    }

    StopSignLine top;
    StopSignLine topRight;
    StopSignLine right;
    StopSignLine botRight;
    StopSignLine bot;
    StopSignLine botLeft;
    StopSignLine left;
    StopSignLine topLeft;
};

class CalibStopSign
{
public:
    CalibStopSign();
    GC_STATUS Load (const std::string jsonCalString );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS CalcHomographies();
    GC_STATUS Calibrate( const cv::Mat &img, const std::string &controlJson, std::string &err_msg );
    GC_STATUS AdjustCalib( const cv::Point2d ptLft, const cv::Point2d ptRgt, const cv::Rect searchROI );
    GC_STATUS AdjustStopSignForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );

    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawOverlay( const cv::Mat &img, cv::Mat &result, const bool drawCalibScale, const bool drawCalibGrid,
                           const bool drawMoveROIs, const bool drawSearchROI , const bool drawTargetSearchROI );
    GC_STATUS DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg );
    GC_STATUS GetCalibParams( std::string &calibParams );

    void clear();

    /**
     * @brief Returns a vector of search lines along which an image is search for a water level line.
     * @return A vector of LineEnds that represent search lines
     */
    std::vector< LineEnds > &SearchLineSet() { return model.searchLineSet; }
    std::string ControlJson() { return model.controlJson; }
    cv::Point2d &MoveOffset() { return moveOffset; }
    CalibModelSymbol &Model() { return model; }
    StopsignSearch &SearchObj() { return stopsignSearch; }
    GC_STATUS GetSearchRegionBoundingRect( cv::Rect &rect );

    cv::Rect &TargetRoi() { return model.targetSearchRegion; }

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;
    CalibModelSymbol model;
    StopsignSearch stopsignSearch;
    cv::Point2d moveOffset;

    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts );
    GC_STATUS GetLineEndPoints( cv::Mat &mask, const cv::Rect rect, cv::Point2d &pt1, cv::Point2d &pt2 );
    GC_STATUS LineIntersection( const StopSignLine line1, const StopSignLine line2, cv::Point2d &r );
    GC_STATUS FindCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines );
    GC_STATUS FindDiagonals( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines );
    GC_STATUS CalcCorners( const OctagonLines octoLines, std::vector< cv::Point2d > &symbolCorners );
    GC_STATUS CalcOctoWorldPoints( const double sideLength, std::vector< cv::Point2d > &pts );
    GC_STATUS CalcSearchLines( const cv::Mat &img, std::vector< cv::Point > &searchLineCorners, std::vector< LineEnds > &searchLines );
    GC_STATUS CreateCalibration( const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts );
    GC_STATUS CalcCenterAngle( const std::vector< cv::Point2d > &pts, cv::Point2d &center, double &angle );
    GC_STATUS CalcGridDrawPoints (std::vector< StopSignLine > &horzLines, std::vector< StopSignLine > &vertLines );
    GC_STATUS GetXEdgeMinDiffX( const double xWorld, cv::Point2d &ptPix, const bool isTopSideY );
    GC_STATUS GetXEdgeMinDiffY( const double yWorld, cv::Point2d &ptPix, const bool isRightSideX );
    GC_STATUS CalcSearchROI( const double zeroOffset, const double botLftPtToLft, const double botLftPtToTop,
                             const double botLftPtToRgt, const double botLftPtToBot, cv::Point2d &lftTop,
                             cv::Point2d &rgtTop, cv::Point2d &lftBot, cv::Point2d &rgtBot );
    GC_STATUS TestCalibration( bool &isValid );
};

} // namespace gc

#endif // CALIBSTOPSIGN_H
