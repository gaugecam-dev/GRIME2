/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** \file gc_types.h
 * @brief An include file with data classes, enums and constants used by the gaugecam libraries.
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2021, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef GC_TYPES_H
#define GC_TYPES_H

#include <string>
#include <limits>
#include <opencv2/core.hpp>

namespace gc
{

#ifdef _WIN32
static const std::string CACHE_FOLDER = "c:/gaugecam/cache/";
#else
static const std::string CACHE_FOLDER = "/var/tmp/gaugecam/cache/";
#endif


/// enum for namespace gc method return values
enum GC_STATUS
{
    GC_EXCEPT = -2,      ///< An exception was thrown
    GC_ERR =    -1,      ///< Error
    GC_OK =      0,      ///< Ok
    GC_WARN =    1       ///< Warning
};

/// enum for namespace gc timestamp sources
enum GC_TIMESTAMP_TYPE
{
    FROM_FILENAME = 0,  ///< Extract timestamp from filename using specified format
    FROM_EXIF,          ///< Get timestamp from image file exif data using specified format
    FROM_EXTERNAL       ///< Pass filename to algorithm using YYYY-MM-DDThh:mm::ss format (ISO)
};

/// enum for draw selections
enum IMG_DISPLAY_OVERLAYS
{
    OVERLAYS_NONE = 0,
    CALIB_SCALE = 1,
    CALIB_GRID = 2,
    TARGET_ROI = 4,
    FINDLINE = 8,
    FEATROIS = 16,
    MOVE_ROIS = 32,
    MOVE_FIND = 64,
    DIAG_ROWSUMS = 128,
    FINDLINE_1ST_DERIV = 256,
    FINDLINE_2ND_DERIV = 512,
    RANSAC_POINTS = 1024,
    SEARCH_ROI = 2048
};

static const double DEFAULT_MIN_LINE_ANGLE = -1.5;                             ///< Default minimum line find angle
static const double DEFAULT_MAX_LINE_ANGLE = 3.5;                              ///< Default maximum line find angle
static const int FIT_LINE_RANSAC_TRIES_TOTAL = 100;                             ///< Fit line RANSAC total tries
static const int FIT_LINE_RANSAC_TRIES_EARLY_OUT = 50;                          ///< Fit line RANSAC early out tries
static const int FIT_LINE_RANSAC_POINT_COUNT = 5;                               ///< Fit line RANSAC early out tries
static const int MIN_DEFAULT_INT = -std::numeric_limits< int >::max();          ///< Minimum value for an integer
static const double MIN_DEFAULT_DBL = -std::numeric_limits< double >::max();    ///< Minimum value for a double
static const int GC_BOWTIE_TEMPLATE_DIM = 56;                                   ///< Default bowtie template size
static const int GC_STOPSIGN_TEMPLATE_DIM = 51;                                 ///< Default bowtie template size
static const int GC_IMAGE_SIZE_WIDTH = 800;                                     ///< Default image width
static const int GC_IMAGE_SIZE_HEIGHT = 600;                                    ///< Default image height
static const double MIN_BOWTIE_FIND_SCORE = 0.55;                               ///< Minimum bow tie sccore

/**
 * @brief Data class to define a line to search an image for a water edge
 */
class LineEnds
{
public:
    /**
     * @brief Constructor to set the line from input values
     * @param ptTop Top point of the line
     * @param ptBot Bottom point of the line
     */
    LineEnds( const cv::Point ptTop, cv::Point ptBot ) : top( ptTop ), bot( ptBot ) {}

    /**
     * @brief Constructor to set invalid point values (uninitialized state)
     */
    LineEnds() :
        top( MIN_DEFAULT_INT, MIN_DEFAULT_INT ),
        bot( MIN_DEFAULT_INT, MIN_DEFAULT_INT )
    {}

    cv::Point top; ///< Top point of the line
    cv::Point bot; ///< Bottom point of the line
};

/**
 * @brief Data class to hold all items needed to define a calibration
 */
class CalibModelBowtie
{
public:
    /**
     * @brief Constructor to set the model to an uninitialized state
     */
    CalibModelBowtie() :
        imgSize( cv::Size( -1, -1 ) ),
        gridSize( cv::Size( -1, -1 ) ),
        moveSearchRegionLft( cv::Rect( -1, -1, -1, -1 ) ),
        moveSearchRegionRgt( cv::Rect( -1, -1, -1, -1 ) ),
        moveSearchROIMultiplier( 0.0 ),
        wholeTargetRegion( cv::Rect( -1, -1, -1, -1 ) )
    {}

    /**
     * @brief Constructor to set the model to a valid state
     * @param gridSz Dimensions of the calibration grid
     * @param pixelPts Vector of pixel points ordered to match the world point vector
     * @param worldPts Vector of world points ordered to match the pixel point vector
     * @param lineEndPts Vector of search lines to be searched for the water line
     * @param mvSrchROILft Left move search region (to search for top-left bowtie)
     * @param mvSrchROIRgt Right move search region (to search for top-right bowtie)
     */
    CalibModelBowtie( cv::Size imageSize,
                cv::Size gridSz,
                std::vector< cv::Point2d > pixelPts,
                std::vector< cv::Point2d > worldPts,
                std::vector< cv::Point > waterLevelSearchCorners,
                std::vector< LineEnds > lineEndPts,
                cv::Rect mvSrchROILft,
                cv::Rect mvSrchROIRgt,
                double moveSearchROIMultiply ) :
        imgSize( imageSize ),
        gridSize( gridSz ),
        pixelPoints( pixelPts ),
        worldPoints( worldPts ),
        waterlineSearchCorners( waterLevelSearchCorners ),
        searchLineSet( lineEndPts ),
        moveSearchRegionLft( mvSrchROILft ),
        moveSearchRegionRgt( mvSrchROIRgt ),
        moveSearchROIMultiplier( moveSearchROIMultiply ),
        wholeTargetRegion( cv::Rect( -1, -1, -1, -1 ) )
    {}

    /**
    * @brief Resets the object to uninitialized values
    */
    void clear()
    {
        controlJson.clear();
        imgSize = cv::Size( -1, -1 );
        gridSize = cv::Size( -1, -1 );
        pixelPoints.clear();
        worldPoints.clear();
        waterlineSearchCorners.clear();
        searchLineSet.clear();
        moveSearchRegionLft = cv::Rect( -1, -1, -1, -1 );
        moveSearchRegionRgt = cv::Rect( -1, -1, -1, -1 );
        moveSearchROIMultiplier = 0.0;
        wholeTargetRegion = cv::Rect( -1, -1, -1, -1 );
    }

    std::string controlJson;                         ///< Json control string
    cv::Size imgSize;                                ///< Dimensions of the calibration image
    cv::Size gridSize;                               ///< Dimensions of the calibration grid
    std::vector< cv::Point2d > pixelPoints;          ///< Vector of pixel points ordered to match the world point vector
    std::vector< cv::Point2d > worldPoints;          ///< Vector of world points ordered to match the pixel point vector
    std::vector< cv::Point > waterlineSearchCorners; ///< Vector of search ROI corners (start at top-left, clockwise
    std::vector< LineEnds > searchLineSet;           ///< Vector of search lines to be searched for the water line
    cv::Rect moveSearchRegionLft;                    ///< Left move search region (to search for top-left bowtie)
    cv::Rect moveSearchRegionRgt;                    ///< Right move search region (to search for top-right bowtie)
    double moveSearchROIMultiplier;                  ///< Move search region multiplier (based on nominal)
    cv::Rect wholeTargetRegion;                      ///< Region within which to perform line and move search
};

class CalibModelSymbol
{
public:
    /**
     * @brief Constructor to set the model to an uninitialized state
     */
    CalibModelSymbol() :
        validCalib( false ),
        imgSize( cv::Size( -1, -1 ) ),
        targetSearchRegion( cv::Rect( -1, -1, -1, -1 ) ),
        facetLength( -1.0 ),
        zeroOffset( 2.0 ),
        center( cv::Point2d( -1.0, -1.0 ) ),
        angle( -9999999.0 )
    {}

    // TODO: Update doxygen
    /**
     * @brief Constructor to set the model to a valid state
     * @param gridSz Dimensions of the calibration grid
     * @param pixelPts Vector of pixel points ordered to match the world point vector
     * @param worldPts Vector of world points ordered to match the pixel point vector
     * @param lineEndPts Vector of search lines to be searched for the water line
     * @param mvSrchROILft Left move search region (to search for top-left bowtie)
     * @param mvSrchROIRgt Right move search region (to search for top-right bowtie)
     */
    CalibModelSymbol( const bool isCalibValid,
                      const cv::Size imageSize,
                      const std::vector< cv::Point2d > pixelPts,
                      const std::vector< cv::Point2d > worldPts,
                      const std::vector< cv::Point > waterLevelSearchCorners,
                      const std::vector< LineEnds > lineEndPts,
                      const cv::Rect symbolSearchROI,
                      const double facetLen,
                      const double zeroOffsetVertical,
                      const cv::Point2d centerPoint,
                      const double symbolAngle ) :
        validCalib( isCalibValid ),
        imgSize( imageSize ),
        pixelPoints( pixelPts ),
        worldPoints( worldPts ),
        waterlineSearchCorners( waterLevelSearchCorners ),
        searchLineSet( lineEndPts ),
        targetSearchRegion( symbolSearchROI ),
        facetLength( facetLen ),
        zeroOffset( zeroOffsetVertical ),
        center( centerPoint ),
        angle( symbolAngle )
    {}

    /**
    * @brief Resets the object to uninitialized values
    */
    void clear()
    {
        validCalib = false;
        controlJson.clear();
        imgSize = cv::Size( -1, -1 );
        pixelPoints.clear();
        worldPoints.clear();
        waterlineSearchCorners.clear();
        searchLineSet.clear();
        targetSearchRegion = cv::Rect( -1, -1, -1, -1 );
        facetLength = -1.0;
        zeroOffset = 2.0;
        center = cv::Point2d( -1.0, -1.0 );
        angle = -9999999.0;
    }

    bool validCalib;
    std::string controlJson;                         ///< Json control string
    cv::Size imgSize;                                ///< Dimensions of the calibration image
    std::vector< cv::Point2d > pixelPoints;          ///< Vector of pixel points ordered to match the world point vector
    std::vector< cv::Point2d > worldPoints;          ///< Vector of world points ordered to match the pixel point vector
    std::vector< cv::Point > waterlineSearchCorners; ///< Vector of search ROI corners (start at top-left, clockwise
    std::vector< LineEnds > searchLineSet;           ///< Vector of search lines to be searched for the water line
    cv::Rect targetSearchRegion;                     ///< Region within which to perform line and move search
    double facetLength;                              ///< Length of a stop sign facet in world units
    double zeroOffset;                               ///< Distance from bottom left stop sign corner to zero vertical position
    cv::Point2d center;                              ///< Center of symbol
    double angle;                                    ///< Angle of symbol
};

/**
 * @brief Data class to hold what is required to perform a water line search
 */
class FindLineParams
{
public:
    /**
     * @brief Constructor to set the object to an uninitialized state
     */
    FindLineParams() :
        datetimeOriginal( std::string( "1955-09-24T12:05:00" ) ),
        datetimeProcessing( std::string( "1955-09-24T12:05:01" ) ),
        timeStampType( FROM_EXIF ),
        timeStampStartPos( -1 )
    {}

    /**
     * @brief Constructor to set the object to a valid state
     * @param timeStampOriginal     Datetime stamp when the image to be search was created
     * @param timeStampProcessing   Datetime stamp when the findline was performed
     * @param imageFilepath         Input image filepath of the image to be searched
     * @param calibConfigFile       Input pixel to world coordinate calibration model filepath
     * @param resultImageFilepath   Optional result image created from input image with found line and move detection overlays
     * @param kalmanParams          Kalman enable and parameters
     */
    FindLineParams( const std::string timeStampOriginal,
                    const std::string timeStampProcessing,
                    const std::string imageFilepath,
                    const std::string calibConfigFile,
                    const GC_TIMESTAMP_TYPE tmStampType,
                    const int tmStampStartPos,
                    const std::string tmStampFormat,
                    const std::string resultImageFilepath = "",
                    const std::string resultCSVFilepath = "" ) :
        datetimeOriginal( timeStampOriginal ),
        datetimeProcessing( timeStampProcessing ),
        imagePath( imageFilepath ),
        calibFilepath( calibConfigFile ),
        resultImagePath( resultImageFilepath ),
        resultCSVPath( resultCSVFilepath ),
        timeStampType( tmStampType ),
        timeStampStartPos( tmStampStartPos ),
        timeStampFormat( tmStampFormat ),
        isStopSignCalib( true ),
        stopSignZeroOffset( 0.0 )
    {}

    void clear()
    {
        datetimeOriginal = std::string( "1955-09-24T12:05:00" );
        datetimeProcessing = std::string( "1955-09-24T12:05:01" );
        imagePath.clear();
        resultImagePath.clear();
        resultCSVPath.clear();
        timeStampType = FROM_EXIF;
        timeStampStartPos = -1;
        timeStampFormat.clear();
        calibFilepath.clear();
        isStopSignCalib = true;
        stopSignZeroOffset = 0.0;
        calibControlString.clear();
    }

    // timestamp in ISO 8601 DateTime format
    // e.g. 1955-09-24T12:05:00Z00:00
    std::string datetimeOriginal;       ///< Datetime stamp when the image to be search was created
    std::string datetimeProcessing;     ///< Datetime stamp when the findline was performed
    std::string imagePath;              ///< Input image filepath of the image to be searched
    std::string calibFilepath;          ///< Input pixel to world coordinate calibration model filepath
    std::string resultImagePath;        ///< Optional result image created from input image with found line and move detection overlays
    std::string resultCSVPath;          ///< Optional result csv file path to hold timestamps and stage measurements
    GC_TIMESTAMP_TYPE timeStampType;    ///< Specifies where to get timestamp (filename, exif, or dateTimeOriginal)
    int timeStampStartPos;              ///< start position of timestamp string in filename (not whole path)
    std::string timeStampFormat;        ///< Format of the timestamp string, e.g. YYYY-MM-DDThh:mm::ss
    bool isStopSignCalib;               ///< True = stopsign, false = bowtie
    double stopSignZeroOffset;          ///< Offset from bottom left stop sign point to stage=0.0
    std::string calibControlString;     ///< Calibration string needed for continuous stopsign calibration
};

/**
 * @brief Data class that holds the result of a found line
 */
class FindPointSet
{
public:
    /**
     * @brief Constructor to set the object to an uninitialized state
     */
    FindPointSet() :
        anglePixel( MIN_DEFAULT_DBL ),
        angleWorld( MIN_DEFAULT_DBL ),
        lftPixel( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        lftWorld( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        ctrPixel( cv::Point2d( 0.0, 0.0 ) ),
        ctrWorld( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        rgtPixel( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        rgtWorld( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) )
    {}

    /**
     * @brief Constructor to set the object to a valid state
     * @param anglePixel    Angle of the found line (pixels)
     * @param angleWorld    Angle of the found line (world)
     * @param leftPixel     Left most pixel coordinate position of the found line
     * @param leftWorld     Left most world coordinate position of the found line
     * @param centerPixel   Center pixel coordinate position of the found line
     * @param centerWorld   Center world coordinate position of the found line
     * @param rightPixel    Right most pixel coordinate position of the found line
     * @param rightWorld    Right most world coordinate position of the found line
     */
    FindPointSet( const double lineAnglePixel, const double lineAngleWorld,
                  const cv::Point2d leftPixel, const cv::Point2d leftWorld,
                  const cv::Point2d centerPixel, const cv::Point2d centerWorld,
                  const cv::Point2d rightPixel, const cv::Point2d rightWorld ) :
        anglePixel( lineAnglePixel ), angleWorld( lineAngleWorld ), lftPixel( leftPixel ), lftWorld( leftWorld ),
        ctrPixel( centerPixel ), ctrWorld( centerWorld ), rgtPixel( rightPixel ), rgtWorld( rightWorld )
    {}

    /**
     * @brief Reset the object to an uninitialzed state
     */
    void clear()
    {
        anglePixel = MIN_DEFAULT_DBL;
        angleWorld = MIN_DEFAULT_DBL;
        lftPixel = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        lftWorld = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        ctrPixel = cv::Point2d( 0.0, 0.0 );
        ctrWorld = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        rgtPixel = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        rgtWorld = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
    }

    /**
     * @brief Set values to zero for continuous calibration
     */
    void setZero()
    {
        anglePixel = 0.0;
        angleWorld = 0.0;
        lftPixel = cv::Point2d( 0.0, 0.0 );
        lftWorld = cv::Point2d( 0.0, 0.0 );
        ctrPixel = cv::Point2d( 0.0, 0.0 );
        ctrWorld = cv::Point2d( 0.0, 0.0 );
        rgtPixel = cv::Point2d( 0.0, 0.0 );
        rgtWorld = cv::Point2d( 0.0, 0.0 );
    }

    double anglePixel;      ///< Angle of the found line (pixels)
    double angleWorld;      ///< Angle of the found line (world)
    cv::Point2d lftPixel;   ///< Left most pixel coordinate position of the found line
    cv::Point2d lftWorld;   ///< Left most world coordinate position of the found line
    cv::Point2d ctrPixel;   ///< Center pixel coordinate position of the found line
    cv::Point2d ctrWorld;   ///< Center world coordinate position of the found line
    cv::Point2d rgtPixel;   ///< Right most pixel coordinate position of the found line
    cv::Point2d rgtWorld;   ///< Right most world coordinate position of the found line
};

// TODO:  Add doxygen comments
/**
 * @brief Data class to hold the offsets from the original calibration
 */
class CalibOffset
{
public:
    CalibOffset() :
        calibAngle( -9999999.0 ),
        calibCenterPt( cv::Point2d( -1.0, -1.0 ) ),
        offsetAngle( -9999999.0 ),
        offsetCenterPt( cv::Point2d( 0, 0 ) )
    {}

    CalibOffset( const double calAngle, const cv::Point2d calCenter,
                 const double offAngle, const cv::Point2d offCenter ) :
        calibAngle( calAngle ),
        calibCenterPt( calCenter ),
        offsetAngle( offAngle ),
        offsetCenterPt( offCenter )
    {}

    void clear()
    {
        calibAngle = -9999999.0;
        calibCenterPt = cv::Point2d( -1.0, -1.0 );
        offsetAngle = -9999999.0;
        offsetCenterPt = cv::Point2d( -1.0, -1.0 );
    }

    double calibAngle;                      ///< Calibration angle
    cv::Point2d calibCenterPt;              ///< Calibration center point
    double offsetAngle;                     ///< Calibration offset angle
    cv::Point2d offsetCenterPt;             ///< Calibration offset center point
};

/**
 * @brief Data class to hold the results of a search calculation for both water level and move detection
 */
class FindLineResult
{
public:
    /**
     * @brief Constructor sets the object to an uninitialized state
     */
    FindLineResult()
    {
        clear();
    }

    /**
     * @brief Constructor to set the object to a valid state
     * @param findOk                true=Successful find, false=Failed find
     * @param adjustedWaterLevel    World coordinate water level adjust for any detected motion of the calibration target
     * @param lineEndPoints         Found water level line
     * @param moveRefPoints         Line between the move targets at the time of calibration
     * @param moveFoundPoints       Line between the move targets at the time of the current line find
     * @param moveOffsetPoints      Offset between the moveRef and the moveFound lines
     * @param calibOffsets          Calibration offset center and angle (along with original center and angle)
     * @param lineFoundPts          Water line points used to calculate the found water level line
     * @param rowSumDiag            Find line row sums vector of vectors of points
     * @param oneDerivDiag          Find line row sums first derivative vector of vectors of points
     * @param twoDerivDiag          Find line row sums second derivative vector of vectors of points
     * @param messages              Vector of strings with messages about the line find
     */
    FindLineResult( const bool findOk,
                    const std::string captureTime,
                    const std::string illumination_state,
                    const cv::Point2d adjustedWaterLevel,
                    const FindPointSet lineEndPoints,
                    const FindPointSet moveRefPoints,
                    const FindPointSet moveFoundPoints,
                    const FindPointSet moveOffsetPoints,
                    const CalibOffset calOffsets,
                    const std::vector< cv::Point2d > lineFoundPts,
                    const std::vector< std::vector< cv::Point > > rowSumDiag,
                    const std::vector< std::vector< cv::Point > > oneDerivDiag,
                    const std::vector< std::vector< cv::Point > > twoDerivDiag,
                    const std::vector< std::string > messages ) :
        findSuccess( findOk ),
        timestamp( captureTime ),
        illum_state( illumination_state ),
        waterLevelAdjusted( adjustedWaterLevel ),
        calcLinePts( lineEndPoints ),
        refMovePts( moveRefPoints ),
        foundMovePts( moveFoundPoints ),
        offsetMovePts( moveOffsetPoints ),
        calibOffsets( calOffsets ),
        foundPoints( lineFoundPts ),
        diagRowSums( rowSumDiag ),
        diag1stDeriv( oneDerivDiag ),
        diag2ndDeriv( twoDerivDiag ),
        calibReprojectOffset_x( -9999999.0 ),
        calibReprojectOffset_y( -9999999.0 ),
        calibReprojectOffset_dist( -9999999.0 ),
        msgs( messages )
    {
    }

    /**
     * @brief Reset the object to an uninitialzed state
     */
    void clear()
    {
        findSuccess = false;
        timestamp = std::string( "1955-09-24T12:05:00" );
        illum_state = "N/A";
        waterLevelAdjusted = cv::Point2d( -9999999.9, -9999999.9 );
        calcLinePts.clear();
        refMovePts.clear();
        foundMovePts.clear();
        foundPoints.clear();
        offsetMovePts.clear();
        calibOffsets.clear();
        diagRowSums.clear();
        diag1stDeriv.clear();
        diag2ndDeriv.clear();
        calibReprojectOffset_x = -9999999.0;
        calibReprojectOffset_y = -9999999.0;
        calibReprojectOffset_dist = -9999999.0;
        symbolToWaterLineAngle = 0.0;
        msgs.clear();
    }

    bool findSuccess;                       ///< true=Successful find, false=Failed find
    std::string timestamp;                  ///< time of image capture
    std::string illum_state;                ///< illum_state of image capture
    cv::Point2d waterLevelAdjusted;         ///< World coordinate water level adjust for any detected motion of the calibration target
    FindPointSet calcLinePts;               ///< Found water level line
    FindPointSet refMovePts;                ///< Line between the move targets at the time of calibration
    FindPointSet foundMovePts;              ///< Line between the move targets at the time of the current line find
    FindPointSet offsetMovePts;             ///< Offset between the moveRef and the moveFound lines
    CalibOffset calibOffsets;               ///< Calibration offset center and angle (along with original center and angle)
    double symbolToWaterLineAngle;          ///< Angle difference between the stop sign bottom and the waterline
    std::vector< cv::Point2d > foundPoints; ///< Waterline points used to calculate the found water level line
    std::vector< std::vector< cv::Point > > diagRowSums;   ///< Row sums diagnostic lines
    std::vector< std::vector< cv::Point > > diag1stDeriv;  ///< 1st deriv diagnostic lines
    std::vector< std::vector< cv::Point > > diag2ndDeriv;  ///< 2nd deriv diagnostic lines
    double calibReprojectOffset_x;          ///< Reprojection offset x
    double calibReprojectOffset_y;          ///< Reprojection offset y
    double calibReprojectOffset_dist;       ///< Reprojection offset Euclidean distance
    std::vector< std::string > msgs;        ///< Vector of strings with messages about the line find
};

/**
 * @brief Data class to hold calibration settings, find line parameters, and find line results to be
 * written to the image files as metadata on the completion of a line find operation
 */
class FindData
{
public:
    /**
     * @brief Constructor sets the object to an uninitialized state
     */
    FindData() {}

    /**
     * @brief Constructor to set the object to a valid state
     * @param settings  Calibration settings
     * @param params    Find line parameters
     * @param result    Find line results
     */
    FindData( const CalibModelBowtie settings,
              const FindLineParams params,
              const FindLineResult result ) :
        calibSettings( settings ),
        findlineParams( params ),
        findlineResult( result )
    {}

    /**
     * @brief Resets the object to an invalid state
     */
    void clear()
    {
        calibSettings.clear();
        findlineParams.clear();
        findlineResult.clear();
    }

    CalibModelBowtie calibSettings; ///< Calibration settings
    FindLineParams findlineParams;  ///< Find line parameters
    FindLineResult findlineResult;  ///< Find line results
};

} // namespace gc

#endif // GC_TYPES_H
