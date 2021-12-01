#include "log.h"
#include "animate.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;

static const long MAX_GIF_SIZE = 99999999;

namespace gc
{

Animate::Animate() :
    delay_cs( -1 ),
    image_size( -1, -1 )
{
}
GC_STATUS Animate::BeginGIF( const cv::Size imgSize, const int imgCount, const std::string gifFilepath, const int delay_ms )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 20 > imgSize.width || 7000 < imgSize.width ||
             20 > imgSize.height || 7000 < imgSize.height )
        {
            FILE_LOG( logERROR ) << "[Animate::BeginGIF] Invalid image width must be in range 20x20 to 7000x7000. w=" <<
                                    imgSize.width << " h=" << imgSize.height;
            retVal = GC_ERR;
        }
        else if ( 0 > delay_ms || 10000 < delay_ms )
        {
            FILE_LOG( logERROR ) << "[Animate::BeginGIF] Invalid delay must be in range 0-10000. delay=" << delay_ms;
            retVal = GC_ERR;
        }

        // ulong memSize = img.cols * img.rows * imgPaths.size();
        if ( MAX_GIF_SIZE < imgSize.width * imgSize.height * imgCount )
        {
            FILE_LOG( logERROR ) << "[Animate::BeginGIF] GIF too large (" << imgSize.width * imgSize.height * imgCount <<
                                    "). w * h * count must be less than " << MAX_GIF_SIZE;
            retVal = GC_ERR;
        }
        else
        {
            // Set up filename,
            // xw=N, yw=N,
            // framedelay=1...1000 or so,
            // loopcount =0 means infinite, otherwise as specified
            // bitdepth=8,
            // dither=false (most likely, but you pick)
            // each image is an array of bytes 0-255, in order ((R G B A) x xw) x yw)
            // ----------------------------------------------------------------------

            // and then:
            // ---------
            image_size = imgSize;
            delay_cs = delay_ms / 10;
            bool bRet = ganim.GifBegin( &g, gifFilepath.c_str(), image_size.width, image_size.height, delay_cs, 0, 8, false );
            if ( !bRet )
            {
                FILE_LOG( logERROR ) << "[Animate::BeginGIF] Could not initialize gif writer for " << gifFilepath;
                retVal = GC_ERR;
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[Animate::BeginGIF] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Animate::AddImageToGIF( const cv::Mat &img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 0 > delay_cs || 1000 < delay_cs ||
             0 > image_size.width || 7000 < image_size.width ||
             0 > image_size.height || 7000 < image_size.height )
        {
            FILE_LOG( logERROR ) << "[Animate::AddImageToGIF] Gif creation parameters not initialized properly";
            retVal = GC_ERR;
        }
        else if ( img.size() != image_size )
        {
            FILE_LOG( logERROR ) << "[Animate::AddImageToGIF] Invalid image size. Expected: w=" << image_size.width <<
                                    "h=" << image_size.height << " Actual: w=" << img.cols << " h=" << img.rows;
            retVal = GC_ERR;
        }
        else
        {
            Mat rgba;
            if ( CV_8UC1 == img.type() )
            {
                cvtColor( img, rgba, COLOR_GRAY2RGBA );
            }
            else if ( CV_8UC3 == img.type() )
            {
                cvtColor( img, rgba, COLOR_BGR2RGBA );
            }
            else
            {
                FILE_LOG( logERROR ) << "[Animate::AddImageToGIF] Invalid image type. Must be 8-bit gray or bgr image";
                retVal = GC_ERR;
            }
            if ( GC_OK == retVal )
            {
                // For each frame of the animation, set up RGBA-formatted image, then:
                // -------------------------------------------------------------------
                bool bRet = ganim.GifWriteFrame( &g, rgba.data, img.cols, img.rows, delay_cs, 8, false, nullptr );
                if ( !bRet )
                {
                    FILE_LOG( logERROR ) << "[Animate::AddImageToGIF] Could not write image";
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[Animate::AddImageToGIF] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Animate::EndGIF()
{
    GC_STATUS retVal = GC_OK;
    try
    {
        image_size = cv::Size( -1, -1 );
        delay_cs = -1;

        // After all frames, this:
        // -----------------------
        ganim.GifEnd( &g );
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[Animate::EndGIF] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace thrive
