#ifndef CALIBEXECUTIVE_H
#define CALIBEXECUTIVE_H

#include "calib.h"
#include "findsymbol.h"

namespace gc
{

typedef enum CALIB_TYPE
{
    BOWTIE,
    STOPSIGN
} GcCalibType;

class CalibExecutive
{
public:
    CalibExecutive();

private:
    GcCalibType calibType;
    Calib bowtie;
    FindSymbol stopsign;

    GC_STATUS LoadCalib( const std::string jsonFilepath );
    GC_STATUS SaveCalib( const std::string jsonFilepath );
    GC_STATUS Calibrate( const std::string calibTargetImagePath, const GcCalibType calibTargetType );
    GC_STATUS Calibrate( const cv::Mat &calibTargetImage, const GcCalibType calibTargetType );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS GetMoveSearchROIs( std::vector< cv::Rect > &rois );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
