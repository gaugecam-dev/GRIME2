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
    GC_STATUS Load( const std::string jsonCalFilepath );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS Calibrate( const cv::Mat &img, const double octoSideLength,
                         const cv::Rect rect, const std::string &controlJson,
                         std::vector< cv::Point > &searchLineCorners );
    GC_STATUS AdjustStopSignForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );
    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawOverlay( const cv::Mat &img, cv::Mat &result, const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
    GC_STATUS SetStopsignColor( const cv::Scalar color, const double minRange, const double maxRange, cv::Scalar &hsv );
    GC_STATUS GetCalibParams( std::string &calibParams );

    void clear();

    /**
     * @brief Returns a vector of search lines along which an image is search for a water level line.
     * @return A vector of LineEnds that represent search lines
     */
    std::vector< LineEnds > &SearchLineSet() { return model.searchLineSet; }
    std::string ControlJson() { return model.controlJson; }
    CalibModelSymbol &Model() { return model; }
    GC_STATUS GetSearchRegionBoundingRect( cv::Rect &rect );

    cv::Rect &TargetRoi() { return model.targetSearchRegion; }

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;
    CalibModelSymbol model;
    cv::Scalar hsvLow;
    cv::Scalar hsvHigh;
    cv::Scalar hsvLow2;
    cv::Scalar hsvHigh2;
    StopsignSearch stopsignSearch;

    GC_STATUS FindColor( const cv::Mat &img, cv::Mat1b &redMask, std::vector< StopSignCandidate > &symbolCandidates );
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
    GC_STATUS GetLineEquation( const cv::Point2d pt1, const cv::Point2d pt2, double &slope, double &intercept );
    GC_STATUS CalcCenterAngle( const std::vector< cv::Point2d > &pts, cv::Point2d &center, double &angle );
};

} // namespace gc

#endif // CALIBSTOPSIGN_H
