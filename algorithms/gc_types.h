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

/// enum for namespace gc method return values
enum GC_STATUS
{
    GC_EXCEPT = -2,      ///< An exception was thrown
    GC_ERR =    -1,      ///< Error
    GC_OK =      0,      ///< Ok
    GC_WARN =    1       ///< Warning
};

/// enum for namespace gc method return values
enum GC_TIMESTAMP_TYPE
{
    FROM_FILENAME = 0,  ///< Extract timestamp from filename using specified format
    FROM_EXIF,          ///< Get timestamp from image file exif data using specified format
    FROM_EXTERNAL       ///< Pass filename to algorithm using YYYY-MM-DDThh:mm::ss format (ISO)
};

static const double DEFAULT_MIN_LINE_ANGLE = -10.0;                             ///< Default minimum line find angle
static const double DEFAULT_MAX_LINE_ANGLE = 10.0;                              ///< Default maximum line find angle
static const int FIT_LINE_RANSAC_TRIES_TOTAL = 100;                             ///< Fit line RANSAC total tries
static const int FIT_LINE_RANSAC_TRIES_EARLY_OUT = 50;                          ///< Fit line RANSAC early out tries
static const int FIT_LINE_RANSAC_POINT_COUNT = 5;                               ///< Fit line RANSAC early out tries
static const int MIN_DEFAULT_INT = -std::numeric_limits< int >::max();          ///< Minimum value for an integer
static const double MIN_DEFAULT_DBL = -std::numeric_limits< double >::max();    ///< Minimum value for a double
static const int GC_BOWTIE_TEMPLATE_DIM = 56;                                   ///< Default bowtie template size
static const int GC_IMAGE_SIZE_WIDTH = 800;                                     ///< Default image width
static const int GC_IMAGE_SIZE_HEIGHT = 600;                                    ///< Default image height

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
class CalibModel
{
public:
    /**
     * @brief Constructor to set the model to an uninitialized state
     */
    CalibModel() :
        gridSize( cv::Size( -1, -1 ) ),
        moveSearchRegionLft( cv::Rect( -1, -1, -1, -1 ) ),
        moveSearchRegionRgt( cv::Rect( -1, -1, -1, -1 ) )
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
    CalibModel( cv::Size gridSz,
                std::vector< cv::Point2d > pixelPts,
                std::vector< cv::Point2d > worldPts,
                std::vector< LineEnds > lineEndPts,
                cv::Rect mvSrchROILft,
                cv::Rect mvSrchROIRgt ) :
        gridSize( gridSz ),
        pixelPoints( pixelPts ),
        worldPoints( worldPts ),
        searchLines( lineEndPts ),
        moveSearchRegionLft( mvSrchROILft ),
        moveSearchRegionRgt( mvSrchROIRgt )
    {}

    /**
    * @brief Resets the object to uninitialized values
    */
    void clear()
    {
        gridSize = cv::Size( -1, -1 );
        pixelPoints.clear();
        worldPoints.clear();
        searchLines.clear();
        moveSearchRegionLft = cv::Rect( -1, -1, -1, -1 );
        moveSearchRegionRgt = cv::Rect( -1, -1, -1, -1 );
    }

    cv::Size gridSize;                      ///< Dimensions of the calibration grid
    std::vector< cv::Point2d > pixelPoints; ///< Vector of pixel points ordered to match the world point vector
    std::vector< cv::Point2d > worldPoints; ///< Vector of world points ordered to match the pixel point vector
    std::vector< LineEnds > searchLines;    ///< Vector of search lines to be searched for the water line
    cv::Rect moveSearchRegionLft;           ///< Left move search region (to search for top-left bowtie)
    cv::Rect moveSearchRegionRgt;           ///< Right move search region (to search for top-right bowtie)
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
        timeStampFormat( tmStampFormat )
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
        ctrPixel( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        ctrWorld( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        rgtPixel( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) ),
        rgtWorld( cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL ) )
    {}

    /**
     * @brief Constructor to set the object to a valid state
     * @param lineAngle     Angle of the found line
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
        ctrPixel = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        ctrWorld = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        rgtPixel = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
        rgtWorld = cv::Point2d( MIN_DEFAULT_DBL, MIN_DEFAULT_DBL );
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
    {}

    /**
     * @brief Constructor to set the object to a valid state
     * @param findOk                true=Successful find, false=Failed find
     * @param adjustedWaterLevel    World coordinate water level adjust for any detected motion of the calibration target
     * @param lineEndPoints         Found water level line
     * @param moveRefPoints         Line between the move targets at the time of calibration
     * @param moveFoundPoints       Line between the move targets at the time of the current line find
     * @param moveOffsetPoints      Offset between the moveRef and the moveFound lines
     * @param lineFoundPts          Water line points used to calculate the found water level line
     * @param messages              Vector of strings with messages about the line find
     */
    FindLineResult( const bool findOk,
                    const std::string captureTime,
                    const cv::Point2d adjustedWaterLevel,
                    const FindPointSet lineEndPoints,
                    const FindPointSet moveRefPoints,
                    const FindPointSet moveFoundPoints,
                    const FindPointSet moveOffsetPoints,
                    const std::vector< cv::Point2d > lineFoundPts,
                    const std::vector< std::string > messages ) :
        findSuccess( findOk ),
        timestamp( captureTime ),
        waterLevelAdjusted( adjustedWaterLevel ),
        calcLinePts( lineEndPoints ),
        refMovePts( moveRefPoints ),
        foundMovePts( moveFoundPoints ),
        offsetMovePts( moveOffsetPoints ),
        foundPoints( lineFoundPts ),
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
        waterLevelAdjusted = cv::Point2d( -9999999.9, -9999999.9 );
        calcLinePts.clear();
        refMovePts.clear();
        foundMovePts.clear();
        foundPoints.clear();
        offsetMovePts.clear();
        msgs.clear();
    }

    bool findSuccess;                       ///< true=Successful find, false=Failed find
    std::string timestamp;                  ///< time of image capture
    cv::Point2d waterLevelAdjusted;         ///< World coordinate water level adjust for any detected motion of the calibration target
    FindPointSet calcLinePts;               ///< Found water level line
    FindPointSet refMovePts;                ///< Line between the move targets at the time of calibration
    FindPointSet foundMovePts;              ///< Line between the move targets at the time of the current line find
    FindPointSet offsetMovePts;             ///< Offset between the moveRef and the moveFound lines
    std::vector< cv::Point2d > foundPoints; ///< Water line points used to calculate the found water level line
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
    FindData( const CalibModel settings,
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

    CalibModel calibSettings;       ///< Calibration settings
    FindLineParams findlineParams;  ///< Find line parameters
    FindLineResult findlineResult;  ///< Find line results
};

} // namespace gc

#endif // GC_TYPES_H
