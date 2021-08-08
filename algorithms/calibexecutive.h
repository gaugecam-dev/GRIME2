#ifndef CALIBEXECUTIVE_H
#define CALIBEXECUTIVE_H

#include "calib.h"
#include "findcalibgrid.h"
#include "findsymbol.h"

namespace gc
{

class CalibExecParams
{
public:
    CalibExecParams() :
        stopSignFacetLength( -1.0 ),
        drawCalib( false ),
        drawMoveSearchROIs( false ),
        drawWaterLineSearchROI( false )
    {}

    void clear()
    {
        calibType.clear();
        worldPtCSVFilepath.clear();
        stopSignFacetLength = -1.0;
        calibResultJsonFilepath.clear();
        drawCalib = false;
        drawMoveSearchROIs = false;
        drawWaterLineSearchROI = false;
    }

    string calibType;
    string worldPtCSVFilepath;
    double stopSignFacetLength;
    string calibResultJsonFilepath;
    bool drawCalib;
    bool drawMoveSearchROIs;
    bool drawWaterLineSearchROI;

    friend std::ostream &operator<<( std::ostream &out, const CalibExecParams &params ) ;
};

class CalibExecutive
{
public:
    CalibExecutive();

    void clear();
    GC_STATUS Load( const std::string jsonFilepath );
    GC_STATUS Calibrate( const std::string calibTargetImagePath, const std::string jsonParams, cv::Mat &imgResult );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS GetMoveSearchROIs( std::vector< cv::Rect > &rois );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                           const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
    std::vector< LineEnds > &SearchLines();
    std::string &GetCalibType() { return paramsCurrent.calibType; }

private:
    Calib bowTie;
    FindSymbol stopSign;
    FindCalibGrid findCalibGrid;
    CalibExecParams paramsCurrent;
    std::vector< LineEnds > nullSearchLines;    ///< Empty vector of search lines to be searched for the water line

    GC_STATUS CalibrateBowTie( const string imgFilepath, cv::Mat &imgOut );
    GC_STATUS CalibrateStopSign( const string imgFilepath, cv::Mat &imgOut );
    GC_STATUS ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< cv::Point2d > > &worldCoords );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
