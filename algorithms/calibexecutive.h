#ifndef CALIBEXECUTIVE_H
#define CALIBEXECUTIVE_H

#include <string>
#include "caliboctagon.h"

namespace gc
{

class CalibExecParams
{
public:
    CalibExecParams() :
        calibType( "Octagon" ),
        facetLength( -1.0 ),
        zeroOffset( 2.0 ),
        botLftPtToLft( -0.5 ),
        botLftPtToTop( 1.0 ),
        botLftPtToRgt( 1.5 ),
        botLftPtToBot( -3.0 ),
        drawCalibScale( false ),
        drawCalibGrid( false ),
        drawWaterLineSearchROI( false ),
        drawTargetSearchROI( false ),
        targetSearchROI( cv::Rect( -1, -1, -1, -1 ) ),
        lineSearch_lftTop( cv::Point( -1, -1 ) ),
        lineSearch_rgtTop( cv::Point( -1, -1 ) ),
        lineSearch_lftBot( cv::Point( -1, -1 ) ),
        lineSearch_rgtBot( cv::Point( -1, -1 ) )
    {}

    void clear()
    {
        calibType = "Octagon";
        facetLength = -1.0;
        zeroOffset = 2.0;
        botLftPtToLft = -0.5;
        botLftPtToTop = 1.0;
        botLftPtToRgt = 1.5;
        botLftPtToBot = -3.0;
        calibResultJsonFilepath.clear();
        drawCalibScale = false;
        drawCalibGrid = false;
        drawWaterLineSearchROI = false;
        drawTargetSearchROI = false;
        targetSearchROI = cv::Rect( -1, -1, -1, -1 );
        lineSearch_lftTop = cv::Point( -1, -1 );
        lineSearch_rgtTop = cv::Point( -1, -1 );
        lineSearch_lftBot = cv::Point( -1, -1 );
        lineSearch_rgtBot = cv::Point( -1, -1 );
    }

    std::string calibType;
    double facetLength;
    double zeroOffset;
    double botLftPtToLft;                            ///< Distance from bottom left stop sign corner to search ROI left
    double botLftPtToTop;                            ///< Distance from bottom left stop sign corner to search ROI top
    double botLftPtToRgt;                            ///< Distance from bottom left stop sign corner to search ROI right
    double botLftPtToBot;                            ///< Distance from bottom left stop sign corner to search ROI bottom
    std::string calibResultJsonFilepath;
    bool drawCalibScale;
    bool drawCalibGrid;
    bool drawWaterLineSearchROI;
    bool drawTargetSearchROI;
    cv::Rect targetSearchROI;
    cv::Point lineSearch_lftTop;
    cv::Point lineSearch_rgtTop;
    cv::Point lineSearch_lftBot;
    cv::Point lineSearch_rgtBot;

    friend std::ostream &operator<<( std::ostream &out, const CalibExecParams &params ) ;
};


class LineSearchRoi
{
public:
    LineSearchRoi() :
        lftTop( cv::Point( 50, 50 ) ),
        rgtTop( cv::Point( 100, 50 ) ),
        rgtBot( cv::Point( 100, 100 ) ),
        lftBot( cv::Point( 50, 100 ) )
    {}

    LineSearchRoi( const cv::Point lftTop, const cv::Point rgtTop,
                    const cv::Point rgtBot, const cv::Point lftBot ) :
        lftTop( lftTop ), rgtTop( rgtTop ), rgtBot( rgtBot ), lftBot( lftBot )
    {}

    void clear()
    {
        lftTop = cv::Point( 50, 50 );
        rgtTop = cv::Point( 100, 50 );
        rgtBot = cv::Point( 100, 100 );
        lftBot = cv::Point( 50, 100 );
    }

    cv::Point lftTop;
    cv::Point rgtTop;
    cv::Point rgtBot;
    cv::Point lftBot;
};

class CalibJsonItems
{
public:
    CalibJsonItems() :
        facetLength( 0.7 ),
        zeroOffset( 2.0 )
    {}
    CalibJsonItems(     const std::string calibResultJson,
                    const bool enableROI,
                    const cv::Rect rect,
                    const double worldFacetLength,
                    const double worldZeroOffset,
                    const LineSearchRoi searchPoly ) :
        calibVisionResult_json( calibResultJson ),
        useROI( enableROI ),
        roi( rect ),
        facetLength( worldFacetLength ),
        zeroOffset( worldZeroOffset ),
        lineSearchPoly( searchPoly )
    {}

    void clear()
    {
        calibVisionResult_json.clear();
        useROI = false;
        roi = cv::Rect( -1, -1, -1, -1 );
        facetLength = -1.0;
        zeroOffset = 0.0;
        lineSearchPoly.clear();
    }

    std::string calibVisionResult_json;
    bool useROI;
    cv::Rect roi;
    double facetLength;
    double zeroOffset;
    LineSearchRoi lineSearchPoly;
};

class CalibExecutive
{
public:
    CalibExecutive();

    void clear();
    bool isCalibrated() { return octagon.isCalibrated(); }
    GC_STATUS Load( const std::string jsonFilepath, const cv::Mat &img );
    GC_STATUS CalibSaveOctagon();
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams,
                         double &rmseDist, double &rmseX, double &rmseY, std::string &err_msg );
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                         double &rmseDist, double &rmseX, double &rmseY, std::string &err_msg,
                         const bool drawAll = false );
    GC_STATUS Recalibrate( const cv::Mat &img, const std::string calibType,
                           double &rmseDist, double &rmseX, double &rmseY, std::string &err_msg );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut , const bool drawAll = false );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale,
                           const bool drawCalibGrid, const bool drawSearchROI, const bool drawTargetROI );
    GC_STATUS DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg );
    GC_STATUS FindMoveTargets( FindPointSet &ptsFound );
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS AdjustOctagonForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );

    CalibModelOctagon &CalibSymbolModel() { return octagon.Model(); }
    std::vector< LineEnds > &SearchLines();
    cv::Rect &TargetRoi();
    std::string &GetCalibType() { return paramsCurrent.calibType; }
    GC_STATUS GetCalibParams( std::string &calibParams );
    GC_STATUS GetTargetSearchROI( cv::Rect &rect );
    GC_STATUS SetAdjustedSearchROI( std::vector< LineEnds > &searchLinesAdj );

    static GC_STATUS FormOctagonCalibJsonString( const CalibJsonItems &items, std::string &json );
    GC_STATUS GetCalibOctagonJsonItems( const std::string &jsonStr, CalibJsonItems &items );
    void EnableAllOverlays()
    {
        paramsCurrent.drawCalibScale = true;
        paramsCurrent.drawCalibGrid = true;
        paramsCurrent.drawWaterLineSearchROI = true;
        paramsCurrent.drawTargetSearchROI = true;
    }

private:
    CalibOctagon octagon;
    CalibExecParams paramsCurrent;
    std::vector< LineEnds > nullSearchLines;    ///< Empty vector of search lines to be searched for the water line
    cv::Rect nullRect = cv::Rect( -1, -1, -1, -1 );

    GC_STATUS CalibrateOctagon( const cv::Mat &img, const std::string &controlJson, std::string &err_msg );

    GC_STATUS FindMoveTargetsOctagon( FindPointSet &ptsFound );
    GC_STATUS MoveRefPointOctagon( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );

    GC_STATUS CalculateRMSE( const std::vector< cv::Point2d > &foundPts, std::vector< cv::Point2d > &reprojectedPts,
                             double &rmseEuclideanDist, double &rmseX, double &rmseY );
    GC_STATUS CalculateRMSE( const cv::Mat &img, double &rmseEuclideanDist, double &rmseX, double &rmseY );
    GC_STATUS SetCalibFromJson( const std::string &jsonParams );
    GC_STATUS TestROIPositions( const int cols, const int rows );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
