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

// TODO: Modify Doxygen comments -- KWC
/**
 * @brief Class to create animations from individual images. The process is as follows:
 *
 * -# Call the CreateCacheFolder() method to ensure a temporary folder is available to add frames.
 * -# Call the ClearCache() method to assure the cache folder is empty.
 * -# Call the AddFrame() method as many times as is required.
 * -# Call the Create() method to create the animation.
 * -# Call the RemoveCacheFolder() to remove the temporary cache folder
 */
class Animate
{
public:
    /**
     * @brief Constructor
     */
    Animate();

    // TODO: Add Doxygen comments -- KWC
    GC_STATUS BeginGIF( const cv::Size imgSize, const int imgCount, const std::string gifFilepath, const int delay_ms );
    GC_STATUS AddImageToGIF( const cv::Mat &img );
    GC_STATUS EndGIF();

private:
    GifAnim ganim;
    GifWriter g;

    int delay_cs;
    cv::Size image_size;
};

} // namespace gc

#endif // CREATEANIMATION_H
