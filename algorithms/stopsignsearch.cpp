#include "log.h"
#include "stopsignsearch.h"
#include <iostream>
#include "opencv2/imgcodecs.hpp"

#ifdef DEBUG_STOPSIGN_TEMPL
#undef DEBUG_STOPSIGN_TEMPL
static const std::string DEBUG_FOLDER = std::string( "/var/tmp/water/" );
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
GC_STATUS StopsignSearch::Init( const int templateDim, const int rotateCnt )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        templates.clear();
        for ( size_t i = 0; i < 8; ++i )
        {
            templates.push_back( StopSignTemplateSet( i * 45 ) );
        }

        retVal = CreatePointTemplates( templateDim, rotateCnt, templates[ 0 ].ptTemplates );
        if ( GC_OK == retVal )
        {
            for ( int i = 1; i < 8; ++i )
            {
                retVal = RotatePointTemplates( i, static_cast< double >( i * 45 ) );
                if ( GC_OK != retVal )
                {
                    break;
                }
            }
        }
#ifdef DEBUG_STOPSIGN_TEMPL
        if ( GC_OK == retVal )
        {
            retVal = CreateTemplateOverlay( DEBUG_FOLDER );
        }
#endif
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::Init] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::RotatePointTemplates( const size_t idx, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( templates.size() <= idx )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] Target template does not exist";
            retVal = GC_ERR;
        }
        else if ( templates[ 0 ].ptTemplates.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] Reference template zero not initialized";
            retVal = GC_ERR;
        }
        else
        {
            Mat rotMask, rotTempl;
            for ( size_t i = 0; i < templates[ 0 ].ptTemplates.size(); ++i )
            {
                templates[ idx ].pointAngle = angle;
                templates[ idx ].ptTemplates.push_back( StopSignTemplate() );
                templates[ idx ].ptTemplates[ i ].angle = templates[ 0 ].ptTemplates[ i ].angle;
                templates[ idx ].ptTemplates[ i ].offset = templates[ 0 ].ptTemplates[ i ].offset;
                retVal = RotateImage( templates[ 0 ].ptTemplates[ i ].mask, rotMask, angle );
                if ( GC_OK == retVal )
                {
                    templates[ idx ].ptTemplates[ i ].mask = rotMask.clone();
                    retVal = RotateImage( templates[ 0 ].ptTemplates[ i ].templ, rotTempl, angle );
                    if ( GC_OK == retVal )
                    {
                        templates[ idx ].ptTemplates[ i ].templ = rotTempl.clone();
                    }
                }
                if ( GC_OK != retVal )
                {
                    break;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] " << e.what();
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
        else if ( 40 > templateDim )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Template dimension must be at least 40";
            retVal = GC_ERR;
        }
        else
        {
            size_t templCnt = rotateCnt * 2 + 1;
            for ( size_t i = 0; i < templCnt; ++i )
            {
                ptTemplates.push_back( StopSignTemplate() );
            }
            int templDim = templateDim + ( 0 == templateDim % 2 ? 1 : 0 );

            double angle;
            Point2d offset;
            Mat mask, templ, templZero, maskZero;
            retVal = DrawCorner( templDim, templZero, maskZero, offset );
            if ( GC_OK == retVal )
            {
                for ( int i = 0; i < rotateCnt; ++i )
                {
                    retVal = RotateImage( maskZero, mask, static_cast< double >( 5 - i ) );
                    if ( GC_OK == retVal )
                    {
                        threshold( mask, mask, 127, 255, THRESH_BINARY );
                        imwrite( "/var/tmp/mask_" + to_string( i ) + ".png", mask );
                        retVal = RotateImage( templZero, templ, static_cast< double >( 5 - i ) );
                        if ( GC_OK == retVal )
                        {
                            imwrite( "/var/tmp/templ_" + to_string( i ) + ".png", templ );
                            ptTemplates[ i ].mask = mask.clone();
                            ptTemplates[ i ].templ = templ.clone();
                            ptTemplates[ i ].angle = static_cast< double >( 5 - i );
                            ptTemplates[ i ].offset = offset;
                            angle = static_cast< double >( ( rotateCnt - 1 ) - ( i + 5 ) );
                            retVal = RotateImage( maskZero, mask, angle );
                            if ( GC_OK == retVal )
                            {
                                threshold( mask, mask, 127, 255, THRESH_BINARY );
                                imwrite( "/var/tmp/mask_" + to_string( rotateCnt + i + 1 ) + ".png", mask );
                                retVal = RotateImage( templZero, templ, angle );
                                if ( GC_OK == retVal )
                                {
                                    imwrite( "/var/tmp/templ_" + to_string( rotateCnt + i + 1 ) + ".png", templ );
                                    ptTemplates[ rotateCnt + i + 1 ].mask = mask.clone();
                                    ptTemplates[ rotateCnt + i + 1 ].templ = templ.clone();
                                    ptTemplates[ rotateCnt + i + 1 ].angle = angle;
                                    ptTemplates[ rotateCnt + i + 1 ].offset = offset;
                                }
                            }
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        break;
                    }
                    // cout << "idx=" << ( i ) << " angle=" << ( 5 - i ) << " idx=" << ( rotateCnt + i + 1 ) << " angle=" << angle << endl;
                }
                if ( GC_OK == retVal )
                {
                    imwrite( "/var/tmp/mask_zero.png", maskZero );
                    imwrite( "/var/tmp/templ_zero.png", templZero );
                    ptTemplates[ rotateCnt ].mask = maskZero.clone();
                    ptTemplates[ rotateCnt ].templ = templZero.clone();
                    ptTemplates[ rotateCnt ].angle = 0.0;
                    ptTemplates[ rotateCnt ].offset = offset;
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
            // imwrite( DEBUG_FOLDER + "mask_001.png", mask );

            contour.clear();
            contour.push_back( Point( ( templateDim >> 1 ), ( templateDim >> 1 ) ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, templateDim >> 1 ) );
            contour.push_back( Point( templateDim >> 1, templateDim >> 1 ) );

            templ = Mat::zeros( Size( tempRotDim, tempRotDim ), CV_8UC1 );
            drawContours( templ( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 128 ), FILLED );
            // imwrite( DEBUG_FOLDER + "templ_000.png", templ );

            templ( rect ) = mask( rect ) - templ( rect );
            // imwrite( DEBUG_FOLDER + "templ_001.png", templ );

            templ( rect ).setTo( 0, templ( rect ) > 200 );
            // imwrite( DEBUG_FOLDER + "templ_002.png", templ );
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
        double angleAdj = 0.0 > angle ? 360.0 + angle : angle;
        Point2d ptCenter = Point2d( static_cast< double >( src.cols ) / 2.0, static_cast< double >( src.rows ) / 2.0 );
        Mat matRotMatrix = getRotationMatrix2D( ptCenter, angleAdj, 1.0 );
        warpAffine( src, dst, matRotMatrix, dst.size(), INTER_CUBIC );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// cornerIdx: 0 = top, left point, followint points move clockwise 1-7
GC_STATUS StopsignSearch::CreateTemplateOverlay( const std::string debugFolder )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( templates.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] Template vector empty";
            retVal = GC_ERR;
        }
        else if ( templates[ 0 ].ptTemplates[ 0 ].mask.empty() || templates[ 0 ].ptTemplates[ 0 ].templ.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] mask and/or template empty";
            retVal = GC_ERR;
        }
        else
        {
            char buf[ 512 ];
            Mat scratch, tempColor = Mat::zeros( Size( templates[ 0 ].ptTemplates[ 0 ].mask.cols * 2,
                                                       templates[ 0 ].ptTemplates[ 0 ].mask.rows ), CV_8UC3 );
            for ( size_t j = 0; j < templates.size(); ++j )
            {
                for ( size_t i = 0; i < templates[ j ].ptTemplates.size(); ++i )
                {
                    cvtColor( templates[ j ].ptTemplates[ i ].templ, scratch, COLOR_GRAY2BGR );
                    scratch.copyTo( tempColor( Rect( 0, 0, templates[ j ].ptTemplates[ 0 ].mask.cols, templates[ j ].ptTemplates[ 0 ].mask.rows ) ) );
                    cvtColor( templates[ j ].ptTemplates[ i ].mask, scratch, COLOR_GRAY2BGR );
                    scratch.copyTo( tempColor( Rect( templates[ j ].ptTemplates[ 0 ].mask.cols, 0,
                                    templates[ j ].ptTemplates[ 0 ].mask.cols, templates[ j ].ptTemplates[ 0 ].mask.rows ) ) );

                    putText( tempColor, "Template", Point( 10, 20 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 255 ), 2 );
                    putText( tempColor, "Mask", Point( templates[ j ].ptTemplates[ 0 ].mask.cols + 10, 20 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 255 ), 2 );
                    sprintf( buf, "Angles pt=%3d templ=%+d", static_cast< int >( templates[ j ].pointAngle ),
                             cvRound( templates[ j ].ptTemplates[ i ].angle ) );
                    putText( tempColor, buf, Point( 10, 40 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 0 ), 1 );
                    sprintf( buf, "%stemplate%03d_%03d.png", debugFolder.c_str(),
                             static_cast< int >( templates[ j ].pointAngle ),
                             cvRound( templates[ j ].ptTemplates[ i ].angle ) + 5 );
                    imwrite( buf, tempColor );
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
