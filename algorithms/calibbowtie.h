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

/** \file calib.h
 * @brief A class to perform pixel to world coordinate calibrations.
 *
 * This file holds the calib class that calculates, saves, loads, and draws pixel to world
 * coordinate calibration models.
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2021, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef CALIBBOWTIE_H
#define CALIBBOWTIE_H

#include <string>
#include <vector>
#include <limits>
#include <opencv2/core/core.hpp>
#include "gc_types.h"

namespace gc
{

/**
 * @brief Performs pixel to world coordinate transforms
 *
 * Provides functionality to create pixel to world and world to pixel
 * coordinate calibration models. The methods expect a set of pixel and world
 * coordinate positions for a 2-column by 4-row array of calibration targets.
 * The class provides methods to calculate the calibration model, save and
 * load models, and manage reference positions to track whether a
 * calibration target has moved since the last calibration, and create
 * overlay images of the model on images.
 *
 * It works well with GaugeCam bow-tie calibration targets that look like this:
 *
 * \image html images/NRmarshDN-12-02-28-11-15_reference_image.jpg "Calibration Target"
 * \image latex images/NRmarshDN-12-02-28-11-15_reference_image.eps "Calibration Target"
 */
class CalibBowtie
{
public:
    /**
     * @brief Constructor
     */
    CalibBowtie() {}

    /**
     * @brief Destructor
     */
    ~CalibBowtie() {}

    /**
     * @brief Clears the calibration object and sets it to an uninitialized state
     * @return None (void)
     */
    void clear();

    /**
     * @brief Create a calibration model from a set of associated pixel and world coordinate points
     * @param pixelPts A vector holding the pixel coordinate points (ordered to match the world points)
     * @param worldPts A vector holding the world coordinate points (ordered to match the pixel points)
     * @param moveSearchROIMultiplier A multiplier to handle more or less camera movement detection
     * @param gridSize The dimensions of the point array (2x4 for standard GaugeCam target)
     * @param imgSize The dimensions of the calibrated image
     * @param resultFilepath Optional filepath for the creation of an image with the calibration overlay
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS Calibrate( const std::vector< cv::Point2d > pixelPts, const std::vector< cv::Point2d > worldPts,
                         const double moveSearchROIMultiplier, const std::string &controlJson, const cv::Size gridSize,
                         const cv::Size imgSize, std::vector< cv::Point > &searchLineCorners );
    /**
     * @brief Load a calibration model from a json file
     *
     * Loads a calibration model that conforms to the following format:
     * \code{.cpp}
     * {
     *   "PixelToWorld":
     *   {
     *     "columns": 2,
     *     "rows": 4,
     *     "points": [
     *       { "pixelX": 408.993, "pixelY": 117.002, "worldX": 10.240, "worldY": 121.750 },
     *       { "pixelX": 666.000, "pixelY": 116.995, "worldX": 70.340, "worldY": 121.750 },
     *       { "pixelX": 410.000, "pixelY": 250.000, "worldX": 10.240, "worldY": 90.100 },
     *       { "pixelX": 663.993, "pixelY": 250.010, "worldX": 70.340, "worldY": 90.100 },
     *       { "pixelX": 411.002, "pixelY": 379.012, "worldX": 10.240, "worldY": 58.450 },
     *       { "pixelX": 661.008, "pixelY": 380.997, "worldX": 70.340, "worldY": 58.450 },
     *       { "pixelX": 412.004, "pixelY": 505.995, "worldX": 10.240, "worldY": 26.800 },
     *       { "pixelX": 659.006, "pixelY": 507.996, "worldX": 70.340, "worldY": 26.800 }
     *     ]
     *   },
     *   "MoveSearchRegions":
     *   {
     *     "Left":  { "x": 353, "y": 61, "width": 112, "height": 112 },
     *     "Right": { "x": 610, "y": 61, "width": 112, "height": 112 }
     *   },
     *   "SearchLines": [
     *       { "topX": 495, "topY": 56, "botX": 494, "botY": 567 },
     *       { "topX": 496, "topY": 56, "botX": 495, "botY": 567 },
     *       { "topX": 497, "topY": 56, "botX": 496, "botY": 567 },
     *                                .
     *                                .
     *                                .
     *       { "topX": 579, "topY": 56, "botX": 575, "botY": 567 },
     *       { "topX": 580, "topY": 56, "botX": 576, "botY": 567 },
     *       { "topX": 581, "topY": 56, "botX": 577, "botY": 567 }
     *   ]
     * }
     * \endcode
     * @param jsonCalibString json string to parse
     * @see Load()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS Load( const std::string &jsonCalibString );

    /**
     * @brief Save the current calibration model to a json file
     *
     * Loads a calibration model of the format shown in the Save()
     * method.
     *
     * @param jsonCalFilepath File to save
     * @see Save()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS Save( const std::string jsonCalFilepath );

    /**
     * @brief Convert a pixel point to a world point
     * @param ptPixel Pixel point to be converted
     * @param ptWorld World point resulting from the conversion
     * @see PixelToWorld()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );

    /**
     * @brief WorldToPixel
     * @param ptWorld World point to be converted
     * @param ptPixel Pixel point resulting from the conversion
     * @see WorldToPixel()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );

    /**
     * @brief Returns the current calibration model properties as a json string
     * @return The json string of the calibration model
     */
    std::string ModelJsonString();

    /**
     * @brief Returns the current calibration model object
     * @return The current calibration model object
     */
    CalibModelBowtie &GetModel() { return m_model; }

    /**
     * @brief Retrieves one of the move search target regions
     * @param isLeft true=Return the left region, false=Return the right region
     * @return A cv::Rect object that holds the specified move search region
     */
    cv::Rect MoveSearchROI( const bool isLeft );

    /**
     * @brief Returns one of the move reference points
     * @param isLeft true=Return the left point, false=Return the right point
     * @return A cv::Point2d object that holds the specified move reference point
     */
    GC_STATUS MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt );

    /**
     * @brief Returns a vector of search lines along which an image is search for a water level line.
     * @return A vector of LineEnds that represent search lines
     */
    std::vector< LineEnds > &SearchLineSet() { return m_model.searchLineSet; }

    /**
     * @brief Returns the current "whole target region" within which the calibration takes place.
     *        All values are set to -1 for the whole image. If the resion is set, the whole
     *        calibration target should fall within the Rect that defines the region.
     * @return A Rect holding the left, top, width, and height of the whole target region
     */
    cv::Rect &TargetRoi() { return m_model.wholeTargetRegion; }

    // TODO: Add doxygen comments
    GC_STATUS GetSearchRegionBoundingRect( cv::Rect &rect );
    CalibModelBowtie &Model() { return m_model; }
    std::string ControlJson() { return m_model.controlJson; }
    GC_STATUS DrawOverlay( const cv::Mat img, cv::Mat &imgOut,
                           const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI );

private:
    cv::Mat m_matHomogPixToWorld;
    cv::Mat m_matHomogWorldToPix;
    // std::vector< double > m_worldToPixParams;

    CalibModelBowtie m_model;
};

}   // namespace gc

#endif // CALIBBOWTIE_H
