#ifndef CALIBEXECUTIVE_H
#define CALIBEXECUTIVE_H

#include "calib.h"
#include "findcalibgrid.h"
#include "findsymbol.h"

namespace gc
{

class CalibExecutive
{
public:
    CalibExecutive();

    GC_STATUS Load( const std::string jsonFilepath );
    GC_STATUS Save( const std::string jsonFilepath );
    GC_STATUS Calibrate( const std::string calibTargetImagePath, const std::string jsonParams, cv::Mat &imgResult );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS GetMoveSearchROIs( std::vector< cv::Rect > &rois );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                           const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
private:
    GcCalibType calibType;
    Calib bowTie;
    FindSymbol stopSign;
    FindCalibGrid m_findCalibGrid;

    GC_STATUS CalibrateBowTie( const string imgFilepath, const string worldCoordsCsv, const string calibJson,
                               cv::Mat &imgOut, const bool drawCalib, const bool drawMoveROIs, const bool drawMoveSearchROIs );
    GC_STATUS CalibrateStopSign( const string imgFilepath, const double facetLength, cv::Mat &imgOut,
                                 const bool drawCalib, const bool drawMoveROIs, const bool drawMoveSearchROIs );
    GC_STATUS ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< cv::Point2d > > &worldCoords );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
