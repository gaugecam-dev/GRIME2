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
    GC_STATUS GetMoveSearchROIs( cv::Rect &rectLeft, cv::Rect &rectRight );
    GC_STATUS SetMoveSearchROIs( const cv::Mat img, const cv::Rect rectLeft, const cv::Rect rectRight );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                           const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );
    GC_STATUS FindMoveTargets( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );

    std::vector< LineEnds > &SearchLines();
    cv::Rect &TargetRoi();
    std::string &GetCalibType() { return paramsCurrent.calibType; }

private:
    Calib bowTie;
    FindSymbol stopSign;
    FindCalibGrid findCalibGrid;
    CalibExecParams paramsCurrent;
    std::vector< LineEnds > nullSearchLines;    ///< Empty vector of search lines to be searched for the water line
    cv::Rect nullRect = cv::Rect( -1, -1, -1, -1 );

    GC_STATUS CalibrateBowTie( const string imgFilepath, cv::Mat &imgOut );
    GC_STATUS CalibrateStopSign( const string imgFilepath );
    GC_STATUS FindMoveTargetsBowTie( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS FindMoveTargetsStopSign( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPointBowTie( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS MoveRefPointStopSign( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< cv::Point2d > > &worldCoords );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
