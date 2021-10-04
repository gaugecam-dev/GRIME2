#ifndef FINDSTAFFGAUGE_H
#define FINDSTAFFGAUGE_H

#include "gc_types.h"

static const int TICK_TEMPL_COUNT = 21;               /**< Number of rotated tik match templates */
static const double TICK_TEMPL_MATCH_MIN_SCORE = 0.1; /**< Minimum tick template match score 0.0 < x < 1.0 */
static const double TICK_TEMPL_ROTATE_INC = CV_PI / 180.0;     /**< Rotation increment for tick match templates */
static const double TICK_TARGET_COUNT_LEFT = 11;
static const double TICK_TARGET_COUNT_RIGHT = 13;
static const int TICK_POINT_COUNT_MIN = 5;

namespace gc
{

typedef enum STAFFGAUGE_TICK_TYPE
{
    BLACK_BOTTOM_LEFT_CORNER,
    BLACK_TOP_RIGHT_POINT
} StaffGaugeTickType;

class TickItem
{
public:
    TickItem() :
        pt( cv::Point2d( -1.0, -1.0 ) ),
        score( -1.0 ),
        y_interval( 0.0 ),
        x_length( 0.0 )
    {}
    TickItem( const cv::Point2d point, const double scoreVal ) :
        pt( point ), score( scoreVal ), y_interval( 0.0 ), x_length( 0.0 ) {}

    cv::Point2d pt;
    cv::Point2d intersectPt;
    double score;
    double y_interval;
    double x_length;
};

class FindStaffGauge
{
public:
    FindStaffGauge();

    GC_STATUS Load( const std::string jsonCalFilepath );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS Find( const cv::Mat &img , cv::Point2d ptTopTickPos,
                    const double distTickToTick, const std::vector< double > tickLengths );
    GC_STATUS Calibrate( const std::vector< cv::Point2d > pixelPts, const std::vector< cv::Point2d > worldPts,
                         const cv::Size gridSize, const cv::Size imgSize, const cv::Mat &img, cv::Mat &imgOut,
                         const bool drawCalib = false, const bool drawMoveROIs = false, const bool drawSearchROI = false );
    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawCalibration( const cv::Mat &img, cv::Mat &result, const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
    void clear();

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;

    std::vector< cv::Mat > m_templates;
    std::vector< TickItem > m_matchItems;
    cv::Mat m_matchSpace;
    cv::Mat m_matchSpaceSmall;
    cv::Point2d linePt1;
    cv::Point2d linePt2;
    std::vector< cv::Point2d > pixelPts;
    std::vector< cv::Point2d > worldPts;

    GC_STATUS CreateTemplates( const cv::Mat &img, const StaffGaugeTickType tickType );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS FindTicks( const cv::Mat &img, StaffGaugeTickType lftTickType );
    GC_STATUS FindTemplates( const cv::Mat &img, const double minScore, const int targetCount, const std::string resultFilepath = "/var/tmp/water/find.png" );
    GC_STATUS MatchTemplate( const int index, const cv::Mat &img, const double minScore, const int numToFind );
    GC_STATUS MatchRefine( const int index, const cv::Mat &img, const double minScore,
                           const int numToFind, TickItem &item );
    GC_STATUS FindTickTipLine( const cv::Mat &img, const std::vector< cv::Point2d > pts, cv::Point2d &pt1, cv::Point2d &pt2 );
    GC_STATUS CalcWorldPts( const cv::Mat &img, cv::Point2d ptTopTickPos, const double distTickToTick, const std::vector< double > tickLengthsLowToHigh );
    GC_STATUS SubpixelPointRefine( const cv::Mat &matchSpace, const cv::Point ptMax, cv::Point2d &ptResult );
    GC_STATUS GetClosestPointOnLine( const cv::Point2d linePt1, const cv::Point2d linePt2, const cv::Point2d pt, cv::Point &ptOnLine );
};

} // namespace gc

#endif // FINDSTAFFGAUGE_H
