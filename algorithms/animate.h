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

    /**
    * @brief After frames have been added to a cache, create a GIF animation
    *
    * @param animationFilepath Filepath of the GIF animation to be created
    * @param fps Frames per second of the animation to be created
    * @param scale Scale of the animation to be created relative to the individual frames
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    GC_STATUS Create( const std::string animationFilepath, const double fps = 2.0, const double scale = 1.0 );

    /**
    * @brief Add a frame (image) to the cache from which the animation is created. The order of the
    *        frames in the animation is the same as the order in which they are added to the cache.
    *
    * @param filename Filepath of the GIF animation to be created
    * @param fps Frames per second of the animation to be created
    * @param scale Scale of the animation to be created relative to the individual frames
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    GC_STATUS AddFrame( std::string filename, cv::Mat frame );

    /**
    * @brief Create the animation frame cache if it does not already exist.
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    static GC_STATUS CreateCacheFolder();

    /**
    * @brief Delete the animation frame cache and all the frames in it if it exists.
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    static GC_STATUS RemoveCacheFolder();

    /**
    * @brief Delete all the files in the animation frame cache.
    * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
    */
    GC_STATUS ClearCache();

private:
    cv::Size frameSize;
};

} // namespace gc

#endif // CREATEANIMATION_H
