#include "log.h"
#include "stopsignsearch.h"
#include <iostream>
#include "opencv2/imgcodecs.hpp"

#ifndef DEBUG_STOPSIGN_TEMPL
#define DEBUG_STOPSIGN_TEMPL
static const std::string DEBUG_FOLDER = "/var/tmp/water/";
#include <boost/filesystem.hpp>
using namespace boost;
namespace fs = filesystem;
#endif

using namespace cv;
using namespace std;

namespace gc
{

StopsignSearch::StopsignSearch()
{
#ifdef DEBUG_STOPSIGN_TEMPL
    if ( !fs::exists( DEBUG_FOLDER ) )
    {
        bool ret = fs::create_directories( DEBUG_FOLDER );
        if ( !ret )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::StopsignSearch] Could not create debug result folder " << DEBUG_FOLDER;
        }
    }
#endif
}
void StopsignSearch::clear()
{
    templates.clear();
}
GC_STATUS StopsignSearch::Find( const cv::Mat &img, std::vector< cv::Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() || templates.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::Find] Templates not created";
            retVal = GC_ERR;
        }
        else
        {
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::Find] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::CreatePointTemplates( const int templateDim, const int rotateCnt, std::vector< StopSignTemplate > &ptTemplates )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 1 > rotateCnt )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Must have more than one rotation template each direction";
            retVal = GC_ERR;
        }
        else
        {
            ptTemplates.clear();
            int templDim = templateDim + ( 0 == templateDim % 2 ? 1 : 0 );

            StopSignTemplate stopSignTemplate;
            Mat templZero, maskZero, mask, templ;
            retVal = DrawCorner( templDim, templZero, maskZero, stopSignTemplate.offset );
            if ( GC_OK == retVal )
            {
                for ( int i = rotateCnt; i > 0; --i )
                {
                    retVal = RotateImage( templZero, templ, static_cast< double >( i ) );
                    if ( GC_OK == retVal )
                    {
                        retVal = RotateImage( maskZero, mask, static_cast< double >( i ) );
                        if ( GC_OK == retVal )
                        {
                            stopSignTemplate.mask = mask.clone();
                            stopSignTemplate.templ = templ.clone();
                            stopSignTemplate.angle = static_cast< double >( i );
                            ptTemplates.push_back( stopSignTemplate );
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                if ( GC_OK == retVal )
                {
                    stopSignTemplate.mask = maskZero.clone();
                    stopSignTemplate.templ = templZero.clone();
                    stopSignTemplate.angle = 0.0;
                    ptTemplates.push_back( stopSignTemplate );

                    for ( int i = 1; i <= rotateCnt; ++i )
                    {
                        retVal = RotateImage( templZero, stopSignTemplate.templ, static_cast< double >( -i ) );
                        if ( GC_OK == retVal )
                        {
                            retVal = RotateImage( maskZero, stopSignTemplate.mask, static_cast< double >( -i ) );
                            if ( GC_OK == retVal )
                            {
                                stopSignTemplate.mask = mask.clone();
                                stopSignTemplate.templ = templ.clone();
                                stopSignTemplate.angle = static_cast< double >( -i );
                                ptTemplates.push_back( stopSignTemplate );
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                if ( GC_OK == retVal )
                {
#ifdef DEBUG_STOPSIGN_TEMPL
                    for ( size_t i = 0; i < ptTemplates.size(); ++i )
                    {
                        imwrite( "/var/tmp/water/mask_" + to_string( i ) + ".png", ptTemplates[ i ].mask );
                        imwrite( "/var/tmp/water/templ_" + to_string( i ) + ".png", ptTemplates[ i ].templ );
                    }
#endif
                }
                else
                {
                    FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Could not rotate templates";
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::DrawCorner( const int templateDim, cv::Mat &templ, cv::Mat &mask, Point2d &center )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 30 > templateDim || 0 == templateDim % 2 )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::DrawCorner] Template dimension too small or not odd: dim=" << templateDim;
            retVal = GC_ERR;
        }
        else
        {
            double blackLineWidth = 20.0;
            int tempRotDim = cvRound( static_cast< double >( templateDim ) * 1.415 );
            tempRotDim += ( 0 == tempRotDim % 2 ? 1 : 0 );
            int rectTopAndLeft = ( tempRotDim - templateDim ) >> 1;
            Rect rect( rectTopAndLeft, rectTopAndLeft, templateDim, templateDim );
            center = Point2d( static_cast< double >( tempRotDim ) / 2.0,
                              static_cast< double >( tempRotDim ) / 2.0 );

            clear();
            mask = Mat::zeros( Size( tempRotDim, tempRotDim ), CV_8UC1 );

            int ortho_dist = cvRound( blackLineWidth * sin( CV_PI * ( 135.0 / 180.0 ) ) / 2.0 );
            int opposite = cvRound( sqrt( 2.0 * blackLineWidth * blackLineWidth ) );

            vector< Point > contour;
            contour.push_back( Point( ( templateDim >> 1 ) - ortho_dist, ( templateDim >> 1 ) - blackLineWidth ) );
            contour.push_back( Point( 0, templateDim - opposite ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, ( ( templateDim >> 1 ) - blackLineWidth ) ) );
            contour.push_back( Point( ( templateDim >> 1 ) - ortho_dist, ( templateDim >> 1 ) - blackLineWidth ) );
            drawContours( mask( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), FILLED );
#ifdef DEBUG_STOPSIGN_TEMPL
            imwrite( "/var/tmp/water/mask_001.png", mask );
#endif

            contour.clear();
            contour.push_back( Point( ( templateDim >> 1 ), ( templateDim >> 1 ) ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, templateDim >> 1 ) );
            contour.push_back( Point( templateDim >> 1, templateDim >> 1 ) );

            templ = Mat::zeros( Size( tempRotDim, tempRotDim ), CV_8UC1 );
            drawContours( templ( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 128 ), FILLED );
#ifdef DEBUG_STOPSIGN_TEMPL
            imwrite( "/var/tmp/water/templ_000.png", templ );
#endif

            templ( rect ) = mask( rect ) - templ( rect );
#ifdef DEBUG_STOPSIGN_TEMPL
            imwrite( "/var/tmp/water/templ_001.png", templ );
#endif

            templ( rect ).setTo( 0, templ( rect ) > 200 );
#ifdef DEBUG_STOPSIGN_TEMPL
            imwrite( "/var/tmp/water/templ_002.png", templ );
#endif
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::DrawCorner] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::RotateImage( const Mat &src, Mat &dst, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d ptCenter = Point2d( static_cast< double >( src.cols ) / 2.0, static_cast< double >( src.rows ) / 2.0 );
        Mat matRotMatrix = getRotationMatrix2D( ptCenter, angle, 1.0 );
        warpAffine( src, dst, matRotMatrix, dst.size(), INTER_CUBIC );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
