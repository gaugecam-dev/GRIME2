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

/** \file findcalibgrid.h
 * @brief Performs searchs in a calibration target image for bowtie targets
 *
 * This file holds the class that searches for the bowties in a calibration
 * target of the following type:
 *
 * \image html images/NRmarshDN-12-02-28-11-15_reference_image.jpg "Calibration Target"
 * \image latex images/NRmarshDN-12-02-28-11-15_reference_image.eps "Calibration Target"
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2021, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef FINDCALIBGRID_H
#define FINDCALIBGRID_H

#include "gc_types.h"
#include <vector>
#include <functional>
#include <opencv2/core.hpp>

using namespace std;

namespace gc
{

static const double TEMPLATE_MATCH_MIN_SCORE = 0.1; /**< Minimum bow tie template match score 0.0 < x < 1.0 */
static const int TARGET_COUNT = 8;                  /**< Number of bowties in a gaugecam calibration target */
static const int TEMPLATE_COUNT = 21;               /**< Number of rotated bowtie match templates */
static const double ROTATE_INC = CV_PI / 180.0;     /**< Rotation increment for bowtie match templates */
static const int CALIB_POINT_ROW_COUNT = 4;         /**< Number of rows of bowties in a gaugecam calibration target */
static const int CALIB_POINT_COL_COUNT = 2;         /**< Number of columns bowties in a gaugecam calibration target */

/**
 * @brief Data class that holds the score and found position of a bowtie
 */
class TemplateBowtieItem
{
public:
    /**
     * @brief Constructor initializes properties to invalid values
     */
    TemplateBowtieItem() : pt( cv::Point2d( -1.0, -1.0 ) ), score( -1.0 ) {}

    /**
     * @brief Constructor initializes properties to user specified values
     * @param point Position of the center of the found bowtie
     * @param scoreVal Score of the template match for the found bowtie
     */
    TemplateBowtieItem( const cv::Point2d point, const double scoreVal ) :
        pt( point ), score( scoreVal ) {}

    /**
     * @brief Destructor
     */
    ~TemplateBowtieItem() {}

    cv::Point2d pt; /**< Position of the center of the found bowtie */
    double score;   /**< Score of the template match for the found bowtie */
};

/**
 * @brief Class to search for bowtie targets for calibration and  for targe movement detection
 */
class FindCalibGrid
{
public:
    /**
     * @brief FindCalibGrid constructor
     */
    FindCalibGrid();

    /**
     * @brief Destructor
     */
    ~FindCalibGrid() {}

    /**
     * @brief Initializes the bowtie search templates
     *
     * Initializes the bowtie search templates creating a template
     * for a series of rotation angles.
     *
     * @param templateDim The template dimension will create an nxn template
     * @param searchImgSize Size of the image to be searched (for probablity space creation)
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS InitBowtieTemplate( const int templateDim , const cv::Size searchImgSize );

    /**
     * @brief Clears the bowtie find object and sets it to an uninitialized state
     * @return None (void)
     */
    void clear();

    /**
     * @brief Search the image for eight calibration targets
     * @param img Image to search
     * @param targetRoi region within which to search for the bow ties
     * @param minScore Minimal acceptable score to accept a "find"
     * @param resultFilepath Optional filepath to save found target positions and scores
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS FindTargets( const cv::Mat &img, const cv::Rect targetRoi, const double minScore, const string resultFilepath = "" );

    /**
     * @brief Get the number of valid points found
     * @return Number of found points
     */
    size_t GetPointCount() { return m_matchItems.size(); }

    /**
     * @brief Set a specified region of interest within which to search for move targeets (bowties)
     * @param img Image within which to search
     * @param rectLeft Rectangle that defines the left search region
     * @param rectRight Rectangle that defines the right search region
     * @see FindMoveTargets(), DrawMoveROIs(), GetMoveTargetROIs()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS SetMoveTargetROI( const cv::Mat &img, const cv::Rect rectLeft, const cv::Rect rectRight );

    /**
     * @brief Search for the move targets in the provided image
     * @param img Image within which to search
     * @param ptLeft The found position of the left region
     * @param ptRight The found position of the right region
     * @see SetMoveTargetROI(), DrawMoveROIs(), GetMoveTargetROIs()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS FindMoveTargets( const cv::Mat &img, const cv::Rect targetRoi, cv::Point2d &ptLeft, cv::Point2d &ptRight );

    // TODO: Fill in FindMoveTargetsStopSign()
    // TODO: This method currently only handles translation and not rotation
#if 0
    GC_STATUS FindMoveTargetsStopSign( const cv::Mat &img, const cv::Rect targetRoi, cv::Point2d &ptLeft, cv::Point2d &ptRight );
#endif

    /**
     * @brief Search for the move targets in the provided image
     * @param img Image within which to search
     * @param targetRoi region within which to search for the bow ties
     * @param ptLeft The found position of the left region
     * @param ptRight The found position of the right region
     * @see SetMoveTargetROI(), DrawMoveROIs(), GetMoveTargetROIs()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS FindMoveTargets( const cv::Mat &img, const cv::Rect targetRoi, cv::Point2d &ptLeft, cv::Point2d &ptRight, const std::string calibType );

    /**
     * @brief Draw the current move regions on the specified image
     * @param img Image onto which to draw the current regions
     * @see SetMoveTarget(), FindMoveTarget(), GetMoveTargetROIs()
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS DrawMoveROIs( cv::Mat &img );

    /**
     * @brief Retrieve the currently defined move target regions
     * @param rectLeft cv::Rect to hold the retrieved left region
     * @param rectRight cv::Rect to hold the retrieved right region
     * @see SetMoveTarget(), FindMoveTarget(), DrawMoveROIs()
     */
    void GetMoveTargetROIs( cv::Rect &rectLeft, cv::Rect &rectRight );

    /**
     * @brief Retrieve a vector of vectors of points that hold the found target points
     * @param pts the vector of vectors of points that hold the found target points
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS GetFoundPoints( vector< vector< cv::Point2d > > &pts );

private:
    std::vector< cv::Mat > m_templates;
    cv::Mat m_matchSpace;
    cv::Mat m_matchSpaceSmall;
    std::vector< TemplateBowtieItem > m_matchItems;
    std::vector< std::vector< TemplateBowtieItem > > m_itemArray;
    cv::Rect m_rectLeftMoveSearch;
    cv::Rect m_rectRightMoveSearch;

    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS MatchTemplate( const int index, const cv::Mat &img, const cv::Rect targetRoi,
                             const double minScore, const int numToFind );
    GC_STATUS MatchRefine( const int index, const cv::Mat &img, const cv::Rect targetRoi,
                           const double minScore, const int numToFind, TemplateBowtieItem &item );

    GC_STATUS SubpixelPointRefine( const cv::Mat &matchSpace, const cv::Point ptMax, cv::Point2d &ptResult );
    GC_STATUS SortPoints( const cv::Size sizeSearchImage );
};

} // namespace gc

#endif // FINDCALIBGRID_H
