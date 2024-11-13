#ifndef CALIBOCTAGON_H
#define CALIBOCTAGON_H

#include "gc_types.h"
#include <opencv2/core.hpp>
#include "octagonsearch.h"

// TODO -- add doxygen comments KWC
namespace gc
{

class OctagonLine
{
public:
    OctagonLine() :
        pt1( cv::Point2d( -1.0, -1.0 ) ),
        pt2( cv::Point2d( -1.0, -1.0 ) )
    {}

    OctagonLine( const cv::Point2d point1, const cv::Point2d point2 ) :
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

    OctagonLine top;
    OctagonLine topRight;
    OctagonLine right;
    OctagonLine botRight;
    OctagonLine bot;
    OctagonLine botLeft;
    OctagonLine left;
    OctagonLine topLeft;
};

class CalibOctagon
{
public:
    CalibOctagon();
    GC_STATUS Load (const std::string jsonCalString );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS CalcHomographies();
    GC_STATUS Calibrate( const cv::Mat &img, const std::string &controlJson, std::string &err_msg );
    GC_STATUS AdjustOctagonForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );

    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawOverlay( const cv::Mat &img, cv::Mat &result, const bool drawCalibScale, const bool drawCalibGrid,
                           const bool drawMoveROIs, const bool drawSearchROI , const bool drawTargetSearchROI );
    GC_STATUS DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg );
    GC_STATUS GetCalibParams( std::string &calibParams );
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt, const bool force = false );

    void clear();

    /**
     * @brief Returns a vector of search lines along which an image is search for a water level line.
     * @return A vector of LineEnds that represent search lines
     */
    std::vector< LineEnds > &SearchLineSet() { return model.searchLineSet; }
    std::string ControlJson() { return model.controlJson; }
    CalibModelOctagon &Model() { return model; }
    OctagonSearch &SearchObj() { return octagonSearch; }
    GC_STATUS GetSearchRegionBoundingRect( cv::Rect &rect );

    cv::Rect &TargetRoi() { return model.targetSearchRegion; }

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;
    CalibModelOctagon model;
    OctagonSearch octagonSearch;
    cv::Point2d moveRefLftPt;
    cv::Point2d moveRefRgtPt;


    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts );
    GC_STATUS GetLineEndPoints( cv::Mat &mask, const cv::Rect rect, cv::Point2d &pt1, cv::Point2d &pt2 );
    GC_STATUS LineIntersection( const OctagonLine line1, const OctagonLine line2, cv::Point2d &r );
    GC_STATUS FindCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines );
    GC_STATUS FindDiagonals( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines );
    GC_STATUS CalcCorners( const OctagonLines octoLines, std::vector< cv::Point2d > &symbolCorners );
    GC_STATUS CalcOctoWorldPoints( const double sideLength, std::vector< cv::Point2d > &pts );
    GC_STATUS CalcSearchLines( const cv::Mat &img, std::vector< cv::Point > &searchLineCorners, std::vector< LineEnds > &searchLines );
    GC_STATUS CreateCalibration( const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts );
    GC_STATUS CalcCenterAngle( const std::vector< cv::Point2d > &pts, cv::Point2d &center, double &angle );
    GC_STATUS CalcGridDrawPoints (std::vector< OctagonLine > &horzLines, std::vector< OctagonLine > &vertLines );
    GC_STATUS GetXEdgeMinDiffX( const double xWorld, cv::Point2d &ptPix, const bool isTopSideY );
    GC_STATUS GetXEdgeMinDiffY( const double yWorld, cv::Point2d &ptPix, const bool isRightSideX );
    GC_STATUS CalcSearchROI( const double botLftPtToLft, const double botLftPtToTop,
                             const double botLftPtToRgt, const double botLftPtToBot, cv::Point2d &lftTop,
                             cv::Point2d &rgtTop, cv::Point2d &lftBot, cv::Point2d &rgtBot );
    GC_STATUS TestCalibration( bool &isValid );
};

} // namespace gc

#endif // CALIBOCTAGON_H
