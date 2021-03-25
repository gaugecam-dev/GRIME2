#include "log.h"
#include "animate.h"
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;

namespace gc
{

Animate::Animate() :
    frameSize( 0, 0 )
{
}
GC_STATUS Animate::RemoveCacheFolder()
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( fs::exists( TEMPORARY_CACHE_FOLDER ) )
            fs::remove_all( TEMPORARY_CACHE_FOLDER );
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::~Annotate::] " << diagnostic_information( e );
    }
    return retVal;
}

GC_STATUS Animate::CreateCacheFolder()
{
    GC_STATUS retVal = GC_OK;
    try
    {
        bool cacheFolderExists = true;
        if ( !fs::exists( TEMPORARY_CACHE_FOLDER ) )
            cacheFolderExists = fs::create_directories( TEMPORARY_CACHE_FOLDER );

        if ( !cacheFolderExists )
        {
            FILE_LOG( logERROR ) << "[Annotate::Annotate] Could not create animation creation cache folder";
        }
        else
        {
            FILE_LOG( logINFO ) << "Cache file create or already existed";
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::Annotate::] " << diagnostic_information( e );
    }
    return retVal;
}
GC_STATUS Animate::CreateRaw( const std::string animationFilepath, const double fps , const double scale )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cout << "Creating animation: " << fs::path( animationFilepath ).filename().string() << " ... ";
        string inputImages = TEMPORARY_CACHE_FOLDER + "raw_image%03d.png";
        string palette = TEMPORARY_CACHE_FOLDER + "palette.png";
        string cmd = "ffmpeg -f image2 -i " + inputImages + " -hide_banner -loglevel error -vf scale=1296:-1, palettegen=max=reserve_transparent=0 -y " + palette;
        int ret = std::system( cmd.c_str() );
        if ( 0 != ret )
        {
            FILE_LOG( logERROR ) << "[Annotate::Create] Failed to create color palette for animation";
            retVal = GC_ERR;
        }
        else
        {
            cmd = "ffmpeg -f image2 -framerate " + to_string( fps ) + " -i " + inputImages +
                    " -c:v libx264 -crf 0 -preset veryslow -c:a libmp3lame -b:v 320k -hide_banner -loglevel error " + TEMPORARY_CACHE_FOLDER + "video.mp4 -y";
            int ret = std::system( cmd.c_str() );
            if ( 0 != ret )
            {
                FILE_LOG( logERROR ) << "[Annotate::Create] Could not create intermediate video for animation";
                retVal = GC_ERR;
            }
            else
            {
                char buf[ 256 ];
                sprintf( buf, "%d", cvRound( static_cast< double >( frameSize.width ) * scale ) );
                cmd = "ffmpeg -y -i " + TEMPORARY_CACHE_FOLDER + "video.mp4 -i " + palette + " -filter_complex \"";
                cmd += "scale=" + string( buf );
                cmd += ":-1:flags=lanczos[x];[x][1:v]paletteuse\" -hide_banner -loglevel error " + animationFilepath;
                int ret = std::system( cmd.c_str() );
                if ( 0 != ret )
                {
                    FILE_LOG( logERROR ) << "[Annotate::Create] Could not create animation";
                    retVal = GC_ERR;
                }
                cout << "done" << endl;
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::Create] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Animate::Create( const std::string animationFilepath, const double fps, const double scale )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cout << "Creating animation: " << fs::path( animationFilepath ).filename().string() << " ... ";
        string inputImages = TEMPORARY_CACHE_FOLDER + "image%03d.png";
        string palette = TEMPORARY_CACHE_FOLDER + "palette.png";
        string cmd = "ffmpeg -f image2 -i " + inputImages + " -hide_banner -loglevel error -vf scale=1296:-1,palettegen -y " + palette;
#ifdef WIN32
        string cmdResult;
        int ret = WinRunCmd::runCmd( cmd.c_str(), cmdResult );
#else
        int ret = std::system( cmd.c_str() );
#endif
        if ( 0 != ret )
        {
            FILE_LOG( logERROR ) << "[Annotate::Create] Failed to create color palette for animation";
            retVal = GC_ERR;
        }
        else
        {
            cmd = "ffmpeg -f image2 -framerate " + to_string( fps ) + " -i " + inputImages +
                    " -c:v libx264 -crf 0 -preset veryslow -c:a libmp3lame -b:v 320k -hide_banner -loglevel error " + TEMPORARY_CACHE_FOLDER + "video.mp4 -y";
#ifdef WIN32
            int ret = WinRunCmd::runCmd( cmd.c_str(), cmdResult );
#else
            int ret = std::system( cmd.c_str() );
#endif
            if ( 0 != ret )
            {
                FILE_LOG( logERROR ) << "[Annotate::Create] Could not create intermediate video for animation";
                retVal = GC_ERR;
            }
            else
            {
                char buf[ 256 ];
                sprintf( buf, "%d", cvRound( static_cast< double >( frameSize.width ) * scale ) );
                cmd = "ffmpeg -y -i " + TEMPORARY_CACHE_FOLDER + "video.mp4 -i " + palette + " -filter_complex \"";
                cmd += "scale=" + string( buf );
                cmd += ":-1:flags=lanczos[x];[x][1:v]paletteuse\" -hide_banner -loglevel error " + animationFilepath;
#ifdef WIN32
               int ret = WinRunCmd::runCmd( cmd.c_str(), cmdResult );
#else
                int ret = std::system( cmd.c_str() );
#endif
                if ( 0 != ret )
                {
                    FILE_LOG( logERROR ) << "[Annotate::Create] Could not create animation";
                    retVal = GC_ERR;
                }
                cout << "done" << endl;
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::Create] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Animate::AddFrame( std::string filename, cv::Mat frame )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        frameSize = frame.size();
        string filepath = TEMPORARY_CACHE_FOLDER + filename;
        bool bRet = imwrite( filepath, frame );
        if ( !bRet )
        {
            FILE_LOG( logERROR ) << "[Annotate::AddFrame] Could not write annotated image to file: " << filepath;
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::AddFrame] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Animate::ClearCache()
{
    GC_STATUS retVal = GC_OK;
    try
    {
        frameSize = Size( 0, 0 );
        fs::path p( TEMPORARY_CACHE_FOLDER );
        if( fs::exists( p ) && fs::is_directory( p ) )
        {
            fs::directory_iterator end;
            for ( fs::directory_iterator it( p ); it != end; ++it )
            {
                if ( fs::is_regular_file( it->status() ) )
                {
                    fs::remove( it->path() );
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Annotate::ClearCache] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace thrive
