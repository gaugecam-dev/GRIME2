#ifndef ANIMATE_H
#define ANIMATE_H

#include "gc_types.h"
#include <opencv2/core.hpp>

#ifdef WIN32
static const std::string TEMPORARY_CACHE_FOLDER = "c:/gaugecam//animate_cache/";
#else
static const std::string TEMPORARY_CACHE_FOLDER = "/var/tmp/gaugecam/animate_cache/";
#endif

namespace gc
{

class Animate
{
public:
    Animate();

    GC_STATUS Create( const std::string animationFilepath, const double fps = 2, const double scale = 1.0 );
    GC_STATUS CreateRaw( const std::string animationFilepath, const double fps = 2, const double scale = 1.0 );
    GC_STATUS AddFrame( std::string filename, cv::Mat frame );

    static GC_STATUS CreateCacheFolder();
    static GC_STATUS RemoveCacheFolder();
    GC_STATUS ClearCache();

private:
    cv::Size frameSize;
};

} // namespace gc

#endif // CREATEANIMATION_H
