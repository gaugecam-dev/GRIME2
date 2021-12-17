#ifndef FINDSYMBOL_H
#define FINDSYMBOL_H

#include "gc_types.h"
#include <opencv2/core.hpp>

// TODO -- add doxygen comments KWC
namespace gc
{

static const double SYMBOL_TEMPL_ANGLE_MAX = 17.5;
static const double SYMBOL_TEMPL_ANGLE_INC = 0.5;

class TemplateFindItem
{
public:
    /**
     * @brief Constructor initializes properties to invalid values
     */
    TemplateFindItem() : pt( cv::Point2d( -1.0, -1.0 ) ), score( -1.0 ) {}

    /**
     * @brief Constructor initializes properties to user specified values
     * @param point Position of the center of the found bowtie
     * @param scoreVal Score of the template match for the found item
     */
    TemplateFindItem( const cv::Point2d point, const double scoreVal ) :
        pt( point ), score( scoreVal ) {}

    /**
     * @brief Destructor
     */
    ~TemplateFindItem() {}

    cv::Point2d pt; /**< Position of the center of the found bowtie */
    double score;   /**< Score of the template match for the found bowtie */
};


class FindSymbol
{
public:
    FindSymbol();
    GC_STATUS Load( const std::string jsonCalFilepath );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS Calibrate( const cv::Mat &img, const cv::Scalar hsvRange_1_start, const cv::Scalar hsvRange_1_end,
                         const cv::Scalar hsvRange_2_start = cv::Scalar( -1 ), const cv::Scalar hsvRange_2_end = cv::Scalar( -1 ) );
    GC_STATUS CreateTemplates( const cv::Mat &img, const cv::Rect templateRect, const cv::Rect searchRect );
    GC_STATUS FindTargets( const cv::Mat &img, const cv::Rect targetRoi, const double minScore, const std::string resultFilepath );
    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawCalibration( const cv::Mat &img, cv::Mat &result, const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
    void clear();

    /**
     * @brief Returns a vector of search lines along which an image is search for a water level line.
     * @return A vector of LineEnds that represent search lines
     */
    std::vector< LineEnds > &SearchLines() { return model.searchLines; }

    cv::Rect &TargetRoi() { return model.wholeTargetRegion; }

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;
    SymbolCalibModel model;

    cv::Rect searchROI;
    std::vector< cv::Mat > templates;
    std::vector< TemplateFindItem > matchItems;

    GC_STATUS FindColorRange( const cv::Mat &img, cv::Mat1b &mask, const cv::Scalar hsvRange_1_start,
                              const cv::Scalar hsvRange_1_end , const cv::Scalar hsvRange_2_start, const cv::Scalar hsvRange_2_end );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS Calibrate( const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts );
    GC_STATUS MatchTemplate( const int index, const cv::Mat &img, const cv::Rect targetRoi, const double minScore, const int numToFind );
    GC_STATUS MatchRefine( const int index, const cv::Mat &img, const cv::Rect targetRoi,
                           const double minScore, const int numToFind, TemplateFindItem &item );
    GC_STATUS SubpixelPointRefine( const cv::Mat &matchSpace, const cv::Point ptMax, cv::Point2d &ptResult );
};

} // namespace gc

#endif // FINDSYMBOL_H
