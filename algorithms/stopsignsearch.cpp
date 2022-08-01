#include "log.h"
#include "stopsignsearch.h"
#include <random>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "bresenham.h"

#ifndef DEBUG_STOPSIGN_TEMPL
#define DEBUG_STOPSIGN_TEMPL
#include <boost/filesystem.hpp>
using namespace boost;
namespace fs = filesystem;
#endif

#ifdef DEBUG_STOPSIGN_LINEFIT
#undef DEBUG_STOPSIGN_LINEFIT
#endif

static const std::string DEBUG_FOLDER = std::string( "/var/tmp/water/" );
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
            Mat matIn = img;
            if ( img.type() == CV_8UC3 )
                cvtColor( img, matIn, COLOR_BGR2GRAY );

            GaussianBlur( matIn, matIn, Size( 5, 5 ), 9.0 );
            medianBlur( matIn, matIn, 17 );
            // imwrite( "/var/tmp/water/template_stopsign_gauss.png", matIn );

            cv::Ptr< CLAHE > clahe = createCLAHE( 1.0 );
            clahe->apply( matIn, matIn );
            // imwrite( "/var/tmp/water/template_stopsign_clahe.png", matIn );

            Mat color;
            if ( img.type() == CV_8UC1 )
                cvtColor( img, color, COLOR_BGR2GRAY );
            else
                img.copyTo( color );

            pts.clear();
            Mat response;
            for ( size_t j = 0; j < templates.size(); ++j )
            {
                Point2d maxMaxPt;
                double maxMaxVal = -9999999;
                for ( size_t i = 0; i < templates[ j ].ptTemplates.size(); ++i )
                {
                    matchTemplate( matIn, templates[ j ].ptTemplates[ i ].templ, response, TM_CCORR_NORMED, templates[ j ].ptTemplates[ i ].mask );

                    double maxVal;
                    Point maxPt;
                    minMaxLoc( response, nullptr, &maxVal, nullptr, &maxPt );
                    if ( maxVal > maxMaxVal )
                    {
                        maxMaxVal = maxVal;
                        maxMaxPt = Point2d( maxPt ) + templates[ j ].ptTemplates[ i ].offset;
                    }
                }
                if ( 0.0 < maxMaxVal )
                {
                    pts.push_back( maxMaxPt );
                    line( color, Point( cvRound( maxMaxPt.x - 10 ), cvRound( maxMaxPt.y ) ),
                          Point( cvRound( maxMaxPt.x + 10 ), cvRound( maxMaxPt.y ) ), Scalar( 0, 255, 0 ), 1 );
                    line( color, Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y - 10 ) ),
                          Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y + 10 ) ), Scalar( 0, 255, 0 ), 1 );
                }
            }
            retVal = RefineFind( img, pts );
            // imwrite( "/var/tmp/water/template_stopsign_find.png", color );
        }
        if ( 8 != pts.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::Find] Found only " << pts.size() << " points";
            retVal = GC_ERR;
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::Find] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}


GC_STATUS StopsignSearch::ShortenLine( const LineEnds &a, const double newLengthPercent, LineEnds &newLine )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double lineLength = sqrt( ( a.top.x - a.bot.x ) * ( a.top.x - a.bot.x ) +
                                  ( a.top.y - a.bot.y ) * ( a.top.y - a.bot.y ) );
        double newLength = lineLength * ( newLengthPercent + ( 1.0 - newLengthPercent ) / 2 );

        retVal = AdjustLineLength( a, newLength, newLine );
        if ( GC_OK == retVal )
        {
            newLength = lineLength * newLengthPercent;
            retVal = AdjustLineLength( LineEnds( newLine.bot, newLine.top ), newLength, newLine );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::ShortenLine] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::AdjustLineLength( const LineEnds &a, const double newLength, LineEnds &newLine )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        newLine.top = a.top;
        double lenAB = sqrt( pow( a.top.x - a.bot.x, 2.0 ) + pow( a.top.y - a.bot.y, 2.0 ) );
        newLine.bot.x = cvRound( a.top.x + ( ( a.top.x - a.bot.x ) / lenAB ) * newLength );
        newLine.bot.y = cvRound( a.top.y + ( ( a.top.y - a.bot.y ) / lenAB ) * newLength );
        newLine.bot.x = cvRound( a.top.x + ( ( a.bot.x - a.top.x ) / lenAB ) * newLength );
        newLine.bot.y = cvRound( a.top.y + ( ( a.bot.y - a.top.y ) / lenAB ) * newLength );

        // lenAB = sqrt(pow(A.x - B.x, 2.0) + pow(A.y - B.y, 2.0));
        // C.x = B.x + (B.x - A.x) / lenAB * length;
        // C.y = B.y + (B.y - A.y) / lenAB * length;
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::AdjustLineLength] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::RefineFind( const Mat &img, vector< Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] Reference image empty";
            retVal = GC_ERR;
        }
        else if ( 8 != pts.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] Need 8 points, but got only " << pts.size();
            retVal = GC_ERR;
        }
        else
        {
            Mat img8u;
            if ( img.type() == CV_8UC1 )
            {
                img.copyTo( img8u );
            }
            else
            {
                cvtColor( img, img8u, COLOR_BGR2GRAY );
            }

            Mat edges;
            medianBlur( img8u, edges, 7 );
            Canny( edges, edges, 35, 70 );

            LineEnds lineEnds;
            vector< LineEnds > lineEndSet;
            vector< Point > lineEdges;
            Mat mask = Mat::zeros( img.size(), CV_8UC1 );
            Mat color;
            if ( img.type() == CV_8UC3 )
            {
                img.copyTo( color );
            }
            else
            {
                cvtColor( img, color, COLOR_GRAY2BGR );
            }

            LineEnds lineA;
            vector< LineEnds > lineSet;
            retVal = ShortenLine( LineEnds( pts[ 0 ], pts[ pts.size() - 1 ] ), 0.5, lineA );
            if ( GC_OK == retVal )
            {
                lineSet.push_back( lineA );
                for ( size_t i = 1; i < pts.size(); ++i )
                {
                    retVal = ShortenLine( LineEnds( pts[ i ], pts[ i - 1 ] ), 0.5, lineA );
                    if ( GC_OK == retVal )
                    {
                        lineSet.push_back( lineA );
                    }
                    else
                    {
                        break;
                    }
                }
                if ( GC_OK == retVal )
                {
                    for ( size_t i = 0; i < lineSet.size(); ++i )
                    {
                        line( color, lineSet[ i ].bot, lineSet[ i ].top, Scalar( 0, 0, 255 ), 1 );
                        mask = 0;
                        line( mask, lineSet[ i ].top, lineSet[ i ].bot, Scalar( 255 ), 15 );
                        mask &= edges;
                        // imwrite( "/var/tmp/water/line_edges.png", mask );
                        findNonZero( mask, lineEdges );
                        for ( size_t j = 0; j < lineEdges.size(); ++j )
                        {
                            color.at<Vec3b>( lineEdges[ j ] )[ 0 ] = 0;
                            color.at<Vec3b>( lineEdges[ j ] )[ 1 ] = 255;
                            color.at<Vec3b>( lineEdges[ j ] )[ 2 ] = 255;
                        }
                        retVal = FitLine( lineEdges, lineEnds, img );
                        if ( GC_OK == retVal )
                        {
                            lineEndSet.push_back( lineEnds );
                        }
                        else
                        {
                            break;
                        }
                    }

                    if ( GC_OK == retVal )
                    {
                        retVal = CalcPointsFromLines( lineEndSet, pts );
                        if ( GC_OK == retVal )
                        {
                            for ( size_t i = 0; i < lineEndSet.size(); ++i )
                            {
                                line( color, Point( cvRound( pts[ i ].x - 10 ), cvRound( pts[ i ].y ) ),
                                      Point( cvRound( pts[ i ].x + 10 ), cvRound( pts[ i ].y ) ), Scalar( 255, 0, 0 ), 1 );
                                line( color, Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y - 10 ) ),
                                      Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y + 10 ) ), Scalar( 255, 0, 0 ), 1 );
                                line( color, lineEndSet[ i ].bot, lineEndSet[ i ].top, Scalar( 0, 255, 0 ), 1 );
                            }
                            // imwrite( "/var/tmp/water/nighttime.png", color );
                        }
                    }
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] " << e.what();
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
        templates.push_back( StopSignTemplateSet( 0 ) );
        for ( size_t i = 1; i < 8; ++i )
        {
            templates.push_back( StopSignTemplateSet( 360 - static_cast< int >( i ) * 45 ) );
        }

        retVal = CreatePointTemplates( templateDim, rotateCnt, templates[ 0 ].ptTemplates );
        if ( GC_OK == retVal )
        {
            for ( int i = 1; i < 8; ++i )
            {
                retVal = RotatePointTemplates( i, static_cast< double >( 360 - i * 45 ) );
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
                templates[ idx ].pointAngle = cvRound( angle );
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
                        // imwrite( "/var/tmp/water/mask_" + to_string( i ) + ".png", mask );
                        retVal = RotateImage( templZero, templ, static_cast< double >( 5 - i ) );
                        if ( GC_OK == retVal )
                        {
                            // imwrite( "/var/tmp/water/templ_" + to_string( i ) + ".png", templ );
                            ptTemplates[ i ].mask = mask.clone();
                            ptTemplates[ i ].templ = templ.clone();
                            ptTemplates[ i ].angle = static_cast< double >( 5 - i );
                            ptTemplates[ i ].offset = offset;
                            angle = static_cast< double >( ( rotateCnt - 1 ) - ( i + 5 ) );
                            retVal = RotateImage( maskZero, mask, angle );
                            if ( GC_OK == retVal )
                            {
                                threshold( mask, mask, 127, 255, THRESH_BINARY );
                                // imwrite( "/var/tmp/water/mask_" + to_string( rotateCnt + i + 1 ) + ".png", mask );
                                retVal = RotateImage( templZero, templ, angle );
                                if ( GC_OK == retVal )
                                {
                                    // imwrite( "/var/tmp/water/templ_" + to_string( rotateCnt + i + 1 ) + ".png", templ );
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
                    // imwrite( "/var/tmp/mask_zero.png", maskZero );
                    // imwrite( "/var/tmp/templ_zero.png", templZero );
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
            int blackLineWidth = 10;
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
            // imwrite( DEBUG_FOLDER + "mask_000.png", mask );

            contour.clear();
            contour.push_back( Point( ( templateDim >> 1 ) + ortho_dist, ( templateDim >> 1 ) + blackLineWidth ) );
            contour.push_back( Point( 0, templateDim + opposite ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, ( ( templateDim >> 1 ) + blackLineWidth ) ) );
            contour.push_back( Point( ( templateDim >> 1 ) + ortho_dist, ( templateDim >> 1 ) + blackLineWidth ) );
            drawContours( mask( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 0 ), FILLED );
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

                    line( tempColor, Point2d( templates[ j ].ptTemplates[ i ].offset.x - 10, templates[ j ].ptTemplates[ i ].offset.y ),
                          Point2d( templates[ j ].ptTemplates[ i ].offset.x + 10, templates[ j ].ptTemplates[ i ].offset.y ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y - 10 ),
                          Point2d( templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y + 10 ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x - 10, templates[ j ].ptTemplates[ i ].offset.y ),
                          Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x + 10, templates[ j ].ptTemplates[ i ].offset.y ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y - 10 ),
                          Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y + 10 ), Scalar( 0, 0, 255 ), 1 );
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
GC_STATUS StopsignSearch::FitLine( const std::vector< Point > &pts, LineEnds &lineEnds, const cv::Mat &img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 5 > pts.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::FitLine] At least five points are needed to fit a line";
            retVal = GC_ERR;
        }
        else
        {
            double angle;
            Vec4d lineVec;
            vector< int > indices;
            vector< double > angles;
            vector< Point2d > ptSet;
            vector< LineEnds > validLines;

            fitLine( pts, lineVec, DIST_L2, 0.0, 0.01, 0.01 );
            lineEnds.top.x = cvRound( lineVec[ 2 ] + ( lineVec[ 0 ] * -lineVec[ 2 ] ) );
            lineEnds.top.y = cvRound( lineVec[ 3 ] + ( lineVec[ 1 ] * -lineVec[ 2 ] ) );
            lineEnds.bot.x = cvRound( lineVec[ 2 ] + ( lineVec[ 0 ] * ( img.cols - lineVec[ 2 ] - 1 ) ) );
            lineEnds.bot.y = cvRound( lineVec[ 3 ] + ( lineVec[ 1 ] * ( img.cols - lineVec[ 2 ] - 1 ) ) );
            angle = atan2( ( lineEnds.bot.y - lineEnds.top.y ),
                           ( lineEnds.bot.x - lineEnds.top.x ) ) * ( 180.0 / CV_PI );

            double rads = angle * CV_PI / 180.0;
            Point2d pt = Point2d( lineEnds.top.x + cos( rads ) * 100.0, lineEnds.top.y + sin( rads ) * 100 );

            bool isVertical;
            double slope, intercept;
            retVal = GetSlopeIntercept( lineEnds.top, pt, slope, intercept, isVertical );
            if ( GC_OK == retVal )
            {
                if ( isVertical )
                {
                    lineEnds.top.x = cvRound( pt.x );
                    lineEnds.top.y = 0;
                    lineEnds.bot.x = cvRound( pt.x );
                    lineEnds.bot.y = img.rows - 1;
                }
                else
                {
                    lineEnds.top.x = 0;
                    lineEnds.top.y = cvRound( intercept );
                    lineEnds.bot.x = img.cols - 1;
                    lineEnds.bot.y = cvRound( slope * lineEnds.bot.x + intercept );
                    clipLine( img.size(), lineEnds.top, lineEnds.bot );
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::FitLine] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::GetSlopeIntercept( const Point2d one, const Point2d two, double &slope, double &intercept, bool &isVertical )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( numeric_limits< double >::epsilon() > two.x - one.x )
        {
            isVertical = true;
            slope = numeric_limits< double >::max();
            intercept = -9999999;
        }
        else
        {
            isVertical = false;
            slope = ( two.y - one.y ) / ( two.x - one.x );
            intercept = one.y - slope * one.x;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::GetSlopeIntercept] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::GetRandomNumbers( const int low_bound, const int high_bound, const int cnt_to_generate,
                                            vector< int > &numbers, const bool isFirst )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( high_bound - low_bound + 1 < static_cast< int >( numbers.size() ) / 2 )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] Not enough points to find good numbers";
            retVal = GC_ERR;
        }
        else
        {
            default_random_engine randomEngine;
            if ( isFirst )
            {
                auto seed = std::chrono::system_clock::now().time_since_epoch().count(); //seed
                randomEngine.seed( static_cast< unsigned int >( seed ) );
            }

            std::uniform_int_distribution< int > di( low_bound, high_bound ); //distribution

            vector< int > tempVec;
            for ( int i = 0; i < cnt_to_generate; ++i )
                tempVec.push_back( 0 );

            bool foundIt;
            int tries = 10;
            numbers.clear();
            while ( tries > 0 && static_cast< int >( numbers.size() ) < cnt_to_generate )
            {
                std::generate( tempVec.begin(), tempVec.end(), [ & ]{ return di( randomEngine ); } );
                for ( size_t i = 0; i < tempVec.size(); ++i )
                {
                    foundIt = false;
                    for ( size_t j = 0; j < numbers.size(); ++j )
                    {
                        if ( tempVec[ i ] == numbers[ j ] )
                        {
                            foundIt = true;
                            break;
                        }
                    }
                    if ( !foundIt )
                    {
                        numbers.push_back( tempVec[ i ] );
                        if ( static_cast< int >( numbers.size() ) >= cnt_to_generate )
                            break;
                    }
                }
                --tries;
            }
            if ( static_cast< int >( numbers.size() ) < cnt_to_generate )
            {
                FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] Not enough unique numbers found";
                retVal = GC_ERR;
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS StopsignSearch::LineIntersection( const LineEnds line1, const LineEnds line2, cv::Point2d &r )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d x = Point2d( line2.top ) - Point2d( line1.top );
        Point2d d1 = Point2d( line1.bot ) - Point2d( line1.top );
        Point2d d2 = Point2d( line2.bot ) - Point2d( line2.top );

        double cross = d1.x * d2.y - d1.y * d2.x;
        if (abs(cross) < numeric_limits< double >::epsilon() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::LineIntersection] Lines are parallel";
            return GC_ERR;
        }

        double t1 = ( x.x * d2.y - x.y * d2.x ) / cross;
        r = Point2d( line1.top ) + d1 * t1;
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::LineIntersection] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS StopsignSearch::CalcPointsFromLines( const std::vector< LineEnds > lines, std::vector< cv::Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 8 != lines.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CalcPointsFromLines] Need 8 lines, but got only " << lines.size();
            retVal = GC_ERR;
        }
        else
        {
            pts.clear();
            Point2d point;
            for ( size_t i = 1; i < lines.size(); ++i )
            {
                retVal = LineIntersection( lines[ i - 1 ], lines[ i ], point );
                if ( GC_OK == retVal )
                {
                    pts.push_back( point );
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                retVal = LineIntersection( lines[ lines.size() - 1 ], lines[ 0 ], point );
                if ( GC_OK == retVal )
                {
                    pts.push_back( point );
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CalcPointsFromLines] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}


} // namespace gc
