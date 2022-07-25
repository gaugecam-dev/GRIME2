#ifndef CALIBEXECUTIVE_H
#define CALIBEXECUTIVE_H

#include "calibbowtie.h"
#include "findcalibgrid.h"
#include "calibstopsign.h"

namespace gc
{

class CalibExecParams
{
public:
    CalibExecParams() :
        stopSignFacetLength( -1.0 ),
        moveSearchROIGrowPercent( 0 ),
        drawCalibScale( false ),
        drawCalibGrid( false ),
        drawMoveSearchROIs( false ),
        drawWaterLineSearchROI( false ),
        targetSearchROI( cv::Rect( -1, -1, -1, -1 ) ),
        lineSearch_lftTop( cv::Point( -1, -1 ) ),
        lineSearch_rgtTop( cv::Point( -1, -1 ) ),
        lineSearch_lftBot( cv::Point( -1, -1 ) ),
        lineSearch_rgtBot( cv::Point( -1, -1 ) )
    {}

    void clear()
    {
        calibType.clear();
        worldPtCSVFilepath.clear();
        stopSignFacetLength = -1.0;
        moveSearchROIGrowPercent = 0;
        calibResultJsonFilepath.clear();
        drawCalibScale = false;
        drawCalibGrid = false;
        drawMoveSearchROIs = false;
        drawWaterLineSearchROI = false;
        targetSearchROI = cv::Rect( -1, -1, -1, -1 );
        lineSearch_lftTop = cv::Point( -1, -1 );
        lineSearch_rgtTop = cv::Point( -1, -1 );
        lineSearch_lftBot = cv::Point( -1, -1 );
        lineSearch_rgtBot = cv::Point( -1, -1 );
    }

    string calibType;
    string worldPtCSVFilepath;
    double stopSignFacetLength;
    int moveSearchROIGrowPercent;
    string calibResultJsonFilepath;
    bool drawCalibScale;
    bool drawCalibGrid;
    bool drawMoveSearchROIs;
    bool drawWaterLineSearchROI;
    cv::Rect targetSearchROI;
    cv::Point lineSearch_lftTop;
    cv::Point lineSearch_rgtTop;
    cv::Point lineSearch_lftBot;
    cv::Point lineSearch_rgtBot;

    friend std::ostream &operator<<( std::ostream &out, const CalibExecParams &params ) ;
};

class CalibExecutive
{
public:
    CalibExecutive();

    void clear();
    GC_STATUS Load( const std::string jsonFilepath, const cv::Mat &img );
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams,
                         double &rmseDist, double &rmseX, double &rmseY );
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                         double &rmseDist, double &rmseX, double &rmseY );
    GC_STATUS Recalibrate( const cv::Mat &img, const std::string calibType,
                           double &rmseDist, double &rmseX, double &rmseY );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS GetMoveSearchROIs( cv::Rect &rectLeft, cv::Rect &rectRight );
    GC_STATUS SetMoveSearchROIs( const cv::Mat img, const cv::Rect rectLeft, const cv::Rect rectRight );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale,
                           const bool drawCalibGrid, const bool drawMoveROIs, const bool drawSearchROI );
    GC_STATUS FindMoveTargets( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS SetStopsignColorRed();
    GC_STATUS SetStopsignColor( const cv::Scalar color, const double minRange, const double maxRange, cv::Scalar &hsv );
    GC_STATUS AdjustStopSignForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );

    CalibModelBowtie &CalibBowtieModel() { return bowTie.Model(); }
    CalibModelSymbol &CalibSymbolModel() { return stopSign.Model(); }
    std::vector< LineEnds > &SearchLines();
    cv::Rect &TargetRoi();
    std::string &GetCalibType() { return paramsCurrent.calibType; } // BowTie or StopSign
    GC_STATUS GetCalibParams( std::string &calibParams );

private:
    CalibBowtie bowTie;
    CalibStopSign stopSign;
    FindCalibGrid findCalibGrid;
    CalibExecParams paramsCurrent;
    std::vector< LineEnds > nullSearchLines;    ///< Empty vector of search lines to be searched for the water line
    cv::Rect nullRect = cv::Rect( -1, -1, -1, -1 );

    GC_STATUS CalibrateBowTie( const cv::Mat &img, const string &controlJson );
    GC_STATUS CalibrateStopSign( const cv::Mat &img, const string &controlJson );
    GC_STATUS FindMoveTargetsBowTie( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPointBowTie( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS MoveRefPointStopSign( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< cv::Point2d > > &worldCoords );
    GC_STATUS CalculateRMSE( const std::vector< cv::Point2d > &foundPts, std::vector< cv::Point2d > &reprojectedPts,
                             double &rmseEuclideanDist, double &rmseX, double &rmseY );
    GC_STATUS CalculateRMSE( const cv::Mat &img, double &rmseEuclideanDist, double &rmseX, double &rmseY );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
