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

/** \file visapp.h
 * @brief Business logic of the gaugecam waterlevel find system
 *
 * This file holds a class that performs higher level functions that make the
 * the GaugeCam libraries easier to use.
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2020, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef VISAPP_H
#define VISAPP_H

#include "calib.h"
#include "findline.h"
#include "findcalibgrid.h"
#include "metadata.h"

//! GaugeCam classes, functions and variables
namespace gc
{

static const std::string GAUGECAM_VISAPP_VERSION = "0.0.0.1";           ///< GaugeCam executive logic (VisApp) software version

/**
 * @brief Business logic that instantiates objects of the GaugeCam classes and provides methods
 * to make their use more straightforward
 */
class VisApp
{
public:
    /**
     * @brief Constructor, initializes the bowtie templates for target searches and creates scratch folders
     */
    VisApp();

    /**
     * @brief Destructor
     */
    ~VisApp() {}

    /**
     * @brief Retreive the current software version of VisApp (executive logic class)
     * @return String holding the version
     */
    static std::string Version() { return GAUGECAM_VISAPP_VERSION; }
    static void GetExifToolVersion() { MetaData::GetExifToolVersion(); }


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Calibration methods
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
    * @brief Create a calibration model and write it to a calibration file
    *
    * Creates a calibration from an image of a calibration target and a csv file that holds the
    * the world coordinates of the center of the bowties in the calibration target. The results
    * are written to a json file. Optionally, a result overlay of the calibration model on the
    * input image can be created.
    *
    * It should be noted that all the calibration target bowties should be out of the water
    * (the water level must be low) to calibrate properly.
    *
    * @param imgFilepath Filepath of the input image with the calibration target
    * @param worldCoordsCsv Filepath of a csv file that holds the world coordinate
    *                       positions of the centers of the calibration target bowties
    * @param calibJson Filepath of the output json file to which the calibration model is written
    * @param resultImagepath Optional filepath for an image that shows the calibration result
    *                        as an overlay on the input image
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    GC_STATUS Calibrate( const std::string imgFilepath, const std::string worldCoordsCsv,
                         const std::string calibJson, const std::string resultImagepath = "" );

    // TODO: Needs doxygen -- KWC
    GC_STATUS Calibrate( const string imgFilepath, const string worldCoordsCsv,
                         const string calibJson, cv::Mat &imgOut, const bool createOverlay = false );

    /**
     * @brief Set the current calibration from a calibration model json file
     * @param calibJson The filepath of the calibration model json file
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS LoadCalib( const std::string calibJson );

    /**
     * @brief Draw the currently loaded calibration onto an overlay image
     * @param imgMatOut OpenCV Mat of the input image onto which the calibration will be written
     * @param imageMatOut OpenCV Mat of the output image that holds the input image with an overlay of the calibration
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS DrawCalibOverlay( const cv::Mat matIn, cv::Mat &imgMatOut );

    /**
     * @brief Retrieve the current calibration model to a CalibModel object
     * @return The CalibModel
     */
    CalibModel GetCalibModel() { return m_calib.GetModel(); }

    /**
     * @brief Retrieve the current calibration model settings as a json string
     * @return The json string
     */
    std::string CalibString() { return m_calib.ModelJsonString(); }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Findline methods
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * @brief Find the water level in an image specified in the FindLineParams
     * @param params Holds the filepaths and all other parameters need to perform a line find calculation
     * @param timestamp Image capture time
     * @param result Holds the results of the line find calculation
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS CalcLine( const FindLineParams params, FindLineResult &result );

    /**
     * @brief Find the water level in an image specified in the FindLineParams
     * @param params Holds the filepaths and all other parameters need to perform a line find calculation
     *        class member object holds results of the line find calculation
     * @param timestamp Image capture time
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS CalcLine( const FindLineParams params );

    /**
     * @brief Find the water level in the specified image
     * @param img OpenCV mat image to search for the waterline using already set parameters
     * @param timestamp Image capture time
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS CalcLine( const cv::Mat &img, const string timestamp );

    // TODO: Write doxygen comments
    GC_STATUS CalcLine( const FindLineParams params, FindLineResult &result, string &resultJson );
    GC_STATUS GetImageData( const std::string filepath, string &data );
    GC_STATUS GetImageData( const std::string filepath, ExifFeatures &exifFeat );
    GC_STATUS GetImageTimestamp( const std::string filepath, std::string &timestamp );
    GC_STATUS CreateAnimation( const std::string imageFolder, const std::string animationFilepath,
                               const double fps, const double scale );

    /**
     * @brief Convert world coordinates to pixel coordinates using the currently set calibration
     * @param worldPt World coordinate xy position
     * @param pixelPt Point to hold the converted pixel coordinate position
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt );

    /**
     * @brief Create an overlay image with the current found water line
     * @param img Source OpenCV Mat image that was searched for a water line
     * @param imgOut Destination OpenCV Mat image holding the source image with find line overlay
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut );

    /**
     * @brief Create an overlay image with a user specified found water line
     * @param img Source OpenCV Mat image that was searched for a water line
     * @param imgOut Destination OpenCV Mat image holding the source image with find line overlay
     * @param findLineResult User specified found line result object
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const FindLineResult findLineResult );

    /**
     * @brief Set the internal found line position from a user specifed find line result
     * @param findLineResult User specified found line result object
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    void SetFindLineResult( const FindLineResult result );

    /**
     * @brief Get the current found line position
     * @return Internal FindLineResult object
     */
    FindLineResult GetFindLineResult();

    /**
     * @brief Create and/or append find line results fo a csv file
     *
     * A header is added to the csv file when it is created.
     *
     * @param resultCSV Output filepath of the csv file to be created or appended
     * @param imgPath Filepath of the image to which the results apply
     * @param result The results of the waterlevel calculation to be appended
     * @param overwrite true=overwrite the csv file destroying what was previously there
     * false=append the data to the file if exists and create a new one if it does not
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS WriteFindlineResultToCSV( const std::string resultCSV, const std::string imgPath,
                                        const FindLineResult &result, const bool overwrite = false );

    /**
     * @brief Draw both calibration model and line find overlays on a line find image based on its metadata
     * @param imageFilepathIn Input image filepath of the line find image that holds metadata
     * @param imageFilepathOut Output image filepath of the overlay image that is created
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS DrawBothOverlays( const std::string imageFilepathIn, const std::string imageFilepathOut );

private:
    std::string m_calibFilepath;

    Calib m_calib;
    FindLine m_findLine;
    FindLineResult m_findLineResult;
    FindCalibGrid m_findCalibGrid;
    MetaData m_metaData;

    GC_STATUS PixelToWorld( FindPointSet &ptSet );
    GC_STATUS ReadWorldCoordsFromCSV( const std::string csvFilepath, std::vector< std::vector< cv::Point2d > > &worldCoords );
    GC_STATUS FindPtSet2JsonString( const FindPointSet set, const string set_type, string &json );
};

} // namespace gc

#endif // VISAPP_H
