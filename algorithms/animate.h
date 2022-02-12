/** \file visapp.h
 * @brief Class to create animations from images
 *
 * This file holds a class that performs creates animations from individual images
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2021, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef ANIMATE_H
#define ANIMATE_H

#include "gc_types.h"
#include <string>
#include <opencv2/core.hpp>
#include "gifanim/gifanim.h"

#ifdef WIN32
#include "wincmd.h"
static const std::string TEMPORARY_CACHE_FOLDER = "c:/gaugecam/animate_cache/";
#else
static const std::string TEMPORARY_CACHE_FOLDER = "/var/tmp/gaugecam/animate_cache/";
#endif

//! GaugeCam classes, functions and variables
namespace gc
{

/**
 * @brief Class to create animations from individual images. The process is as follows:
 *
 * -# Call the BeginGIF() Sets GIF filepath and other parameter initializations.
 * -# Call the AddImageToGIF() Scales the image and adds it to the GIF.
 * -# Call the EndGIF() Cleans up after GIF has been written
 */
class Animate
{
public:
    /**
     * @brief Constructor
     */
    Animate();

    /**
     * @brief Sets the GIF output filepath and other initializations
     * @param imgSize Expected image size for all frames
     * @param imgCount Count of expected images in the GIF to assure there are enough resources to write the GIF
     * @param gifFilepath Sets the GIF output filepath
     * @param delay_ms Delay in milliseconds between image frames
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     * @see AddImageToGIF(), EndGIF()
     */
    GC_STATUS BeginGIF( const cv::Size imgSize, const int imgCount, const std::string gifFilepath, const int delay_ms );

    /**
     * @brief Adds an image to the GIF initalized by a call to BeginGIF()
     * @param img The image to be added to the GIF
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     * @see BeginGIF(), EndGIF()
     */
    GC_STATUS AddImageToGIF( const cv::Mat &img );

    /**
     * @brief Closes the GIF file initialized by BeginGIF() and frees resources
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     * @see BeginGIF(), AddImageToGIF()
     */
    GC_STATUS EndGIF();

private:
    GifAnim ganim;
    GifWriter g;

    int delay_cs;
    cv::Size image_size;
};

} // namespace gc

#endif // CREATEANIMATION_H
