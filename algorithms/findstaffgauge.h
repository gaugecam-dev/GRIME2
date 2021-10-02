#ifndef FINDSTAFFGAUGE_H
#define FINDSTAFFGAUGE_H

#include "gc_types.h"

static const double TICK_TEMPL_MATCH_MIN_SCORE = 0.1; /**< Minimum bow tie template match score 0.0 < x < 1.0 */
static const int TICK_TEMPL_COUNT = 21;               /**< Number of rotated bowtie match templates */
static const double TICK_TEMPL_ROTATE_INC = CV_PI / 180.0;     /**< Rotation increment for bowtie match templates */

namespace gc
{

typedef enum STAFFGAUGE_TICK_TYPE
{
    BLACK_BOTTOM_LEFT_CORNER,
    BLACK_TOP_RIGHT_POINT
} StaffGaugeTickType;

class FindStaffGauge
{
public:
    FindStaffGauge();

    GC_STATUS Load( const std::string jsonCalFilepath );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS FindTicks( const cv::Mat &img, STAFFGAUGE_TICK_TYPE lftTickType, std::vector< cv::Point2d > &points );
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
    cv::Mat m_matchSpace;
    cv::Mat m_matchSpaceSmall;

    GC_STATUS CreateBlackBotLftCornerTemplates( const cv::Mat &img );
    GC_STATUS CreateBlackTopRgtPointTemplates();
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
};

} // namespace gc

#endif // FINDSTAFFGAUGE_H
