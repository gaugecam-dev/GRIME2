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
        facetLength( -1.0 ),
        zeroOffset( 2.0 ),
        botLftPtToLft( -0.5 ),
        botLftPtToTop( 1.0 ),
        botLftPtToRgt( 1.5 ),
        botLftPtToBot( -3.0 ),
        moveSearchROIGrowPercent( 0 ),
        drawCalibScale( false ),
        drawCalibGrid( false ),
        drawMoveSearchROIs( false ),
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
        calibType.clear();
        worldPtCSVFilepath.clear();
        facetLength = -1.0;
        zeroOffset = 2.0;
        botLftPtToLft = -0.5;
        botLftPtToTop = 1.0;
        botLftPtToRgt = 1.5;
        botLftPtToBot = -3.0;
        moveSearchROIGrowPercent = 0;
        calibResultJsonFilepath.clear();
        drawCalibScale = false;
        drawCalibGrid = false;
        drawMoveSearchROIs = false;
        drawWaterLineSearchROI = false;
        drawTargetSearchROI = false;
        targetSearchROI = cv::Rect( -1, -1, -1, -1 );
        lineSearch_lftTop = cv::Point( -1, -1 );
        lineSearch_rgtTop = cv::Point( -1, -1 );
        lineSearch_lftBot = cv::Point( -1, -1 );
        lineSearch_rgtBot = cv::Point( -1, -1 );
    }

    string calibType;
    string worldPtCSVFilepath;
    double facetLength;
    double zeroOffset;
    double botLftPtToLft;                            ///< Distance from bottom left stop sign corner to search ROI left
    double botLftPtToTop;                            ///< Distance from bottom left stop sign corner to search ROI top
    double botLftPtToRgt;                            ///< Distance from bottom left stop sign corner to search ROI right
    double botLftPtToBot;                            ///< Distance from bottom left stop sign corner to search ROI bottom
    int moveSearchROIGrowPercent;
    string calibResultJsonFilepath;
    bool drawCalibScale;
    bool drawCalibGrid;
    bool drawMoveSearchROIs;
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
        zeroOffset( 2.0 ),
        botLftPtToLft( -0.5 ),
        botLftPtToTop( 1.0 ),
        botLftPtToRgt( 1.5 ),
        botLftPtToBot( -3.0 )
    {}
    CalibJsonItems( const std::string worldPosCsvFile,
                    const std::string calibResultJson,
                    const bool enableROI,
                    const cv::Rect rect,
                    const double moveGrowROIPercent,
                    const double worldFacetLength,
                    const double worldZeroOffset,
                    const double offsetToLft,
                    const double offsetToTop,
                    const double offsetToRgt,
                    const double offsetToBot,
                    const LineSearchRoi searchPoly ) :
        worldTargetPosition_csvFile( worldPosCsvFile ),
        calibVisionResult_json( calibResultJson ),
        useROI( enableROI ),
        roi( rect ),
        moveROIGrowPercent( moveGrowROIPercent ),
        facetLength( worldFacetLength ),
        zeroOffset( worldZeroOffset ),
        botLftPtToLft( offsetToLft ),
        botLftPtToTop( offsetToTop ),
        botLftPtToRgt( offsetToRgt ),
        botLftPtToBot( offsetToBot ),
        lineSearchPoly( searchPoly )
    {}

    void clear()
    {
        worldTargetPosition_csvFile.clear();
        calibVisionResult_json.clear();
        useROI = false;
        roi = cv::Rect( -1, -1, -1, -1 );
        moveROIGrowPercent = 10.0;
        facetLength = -1.0;
        zeroOffset = 0.0;
        botLftPtToBot = -1.0;
        botLftPtToTop = -1.0;
        botLftPtToRgt = -1.0;
        botLftPtToBot = -1.0;
        lineSearchPoly.clear();
    }

    std::string worldTargetPosition_csvFile;
    std::string calibVisionResult_json;
    bool useROI;
    cv::Rect roi;
    double moveROIGrowPercent;
    double facetLength;
    double zeroOffset;
    double botLftPtToLft;                            ///< Distance from bottom left stop sign corner to search ROI left
    double botLftPtToTop;                            ///< Distance from bottom left stop sign corner to search ROI top
    double botLftPtToRgt;                            ///< Distance from bottom left stop sign corner to search ROI right
    double botLftPtToBot;                            ///< Distance from bottom left stop sign corner to search ROI bottom
    LineSearchRoi lineSearchPoly;
};

class CalibExecutive
{
public:
    CalibExecutive();

    void clear();
    GC_STATUS Load( const std::string jsonFilepath, const cv::Mat &img );
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams,
                         double &rmseDist, double &rmseX, double &rmseY, string &err_msg );
    GC_STATUS Calibrate( const cv::Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                         double &rmseDist, double &rmseX, double &rmseY, string &err_msg );
    GC_STATUS Recalibrate( const cv::Mat &img, const std::string calibType,
                           double &rmseDist, double &rmseX, double &rmseY, string &err_msg );
    GC_STATUS PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt );
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );
    GC_STATUS GetMoveSearchROIs( cv::Rect &rectLeft, cv::Rect &rectRight );
    GC_STATUS SetMoveSearchROIs( const cv::Mat img, const cv::Rect rectLeft, const cv::Rect rectRight );
    GC_STATUS DetectMove( std::vector< cv::Point2d > &origPos, std::vector< cv::Point2d > &newPos );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut );
    GC_STATUS DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale, const bool drawCalibGrid,
                           const bool drawMoveROIs, const bool drawSearchROI, const bool drawTargetROI,
                           const cv::Point2d moveOffset = cv::Point2d( 0.0, 0.0 ) );
    GC_STATUS FindMoveTargets( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS AdjustStopSignForRotation( const cv::Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle );

    CalibModelBowtie &CalibBowtieModel() { return bowTie.Model(); }
    CalibModelSymbol &CalibSymbolModel() { return stopSign.Model(); }
    std::vector< LineEnds > &SearchLines();
    cv::Rect &TargetRoi();
    std::string &GetCalibType() { return paramsCurrent.calibType; } // BowTie or StopSign
    GC_STATUS GetCalibParams( std::string &calibParams );
    cv::Point2d GetStopSignMoveOffset() { return stopSign.MoveOffset(); }

    static GC_STATUS FormBowtieCalibJsonString( const CalibJsonItems &items, std::string &json );
    static GC_STATUS FormStopsignCalibJsonString( const CalibJsonItems &items, std::string &json );
    GC_STATUS GetCalibStopsignJsonItems( const std::string &jsonStr, CalibJsonItems &items );

private:
    CalibBowtie bowTie;
    CalibStopSign stopSign;
    FindCalibGrid findCalibGrid;
    CalibExecParams paramsCurrent;
    std::vector< LineEnds > nullSearchLines;    ///< Empty vector of search lines to be searched for the water line
    cv::Rect nullRect = cv::Rect( -1, -1, -1, -1 );

    GC_STATUS CalibrateBowTie( const cv::Mat &img, const string &controlJson, string &err_msg );
    GC_STATUS CalibrateStopSign( const cv::Mat &img, const string &controlJson, string &err_msg );

    GC_STATUS FindMoveTargetsBowTie( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS FindMoveTargetsStopSign( const cv::Mat &img, FindPointSet &ptsFound );
    GC_STATUS MoveRefPointBowTie( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );
    GC_STATUS MoveRefPointStopSign( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );

    GC_STATUS ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< cv::Point2d > > &worldCoords );
    GC_STATUS CalculateRMSE( const std::vector< cv::Point2d > &foundPts, std::vector< cv::Point2d > &reprojectedPts,
                             double &rmseEuclideanDist, double &rmseX, double &rmseY );
    GC_STATUS CalculateRMSE( const cv::Mat &img, double &rmseEuclideanDist, double &rmseX, double &rmseY );
    GC_STATUS SetCalibFromJson( const std::string &jsonParams );
};

} // namespace gc

#endif // CALIBEXECUTIVE_H
