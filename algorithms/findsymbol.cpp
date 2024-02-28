#include "log.h"
#include "findsymbol.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <limits>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <iterator>

#ifdef DEBUG_FIND_CALIB_SYMBOL
#undef DEBUG_FIND_CALIB_SYMBOL
#include <iostream>
#include <opencv2/imgcodecs.hpp>
static const std::string DEBUG_RESULT_FOLDER = "/var/tmp/water/";
#endif

using namespace cv;
using namespace std;
using namespace boost;

namespace gc
{

FindSymbol::FindSymbol()
{

}
void FindSymbol::clear()
{
    matHomogPixToWorld = Mat();
    matHomogWorldToPix = Mat();
    model.clear();
}
GC_STATUS FindSymbol::CreateTemplates( const cv::Mat &img, const cv::Rect templateRect, const cv::Rect searchRect )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat gray;
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Cannot create templates from an empty image";
            retVal = GC_ERR;
        }
        else if ( 0 > templateRect.x || 0 > templateRect.y || 10 > templateRect.width || 10 > templateRect.height ||
                  img.cols < templateRect.x + templateRect.width || img.rows < templateRect.y + templateRect.height )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Invalid template region";
            retVal = GC_ERR;
        }
        else if ( 0 > searchRect.x || 0 > searchRect.y || 100 > searchRect.width || 100 > searchRect.height ||
                  img.cols < searchRect.x + searchRect.width || img.rows < searchRect.y + searchRect.height )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Invalid target search region";
            retVal = GC_ERR;
        }
        else if ( img.type() == CV_8UC3 )
        {
            cvtColor( img, gray, COLOR_BGR2GRAY );
        }
        else if ( img.type() == CV_8UC1 )
        {
            gray = img;
        }
        else
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Image must be an gray scale or BGR image to calibrate with a template match";
            retVal = GC_ERR;
        }
        if ( GC_OK == retVal )
        {
            templates.clear();

            Mat templ = gray( templateRect ).clone();
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "template.png", templ );
#endif
            threshold( templ, templ, 3, 255, THRESH_BINARY | THRESH_OTSU );
            erode( templ, templ, Mat(), Point( -1, -1 ), 2 );
            dilate( templ, templ, Mat(), Point( -1, -1 ), 3 );
            erode( templ, templ, Mat() );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "template_thresh.png", templ );
#endif

            std::vector< cv::Mat > rotatedTemplates;
            std::vector< TemplateFindItem > matchItems;

            Rect rotateArea( templateRect.x - ( templateRect.width >> 1 ),
                             templateRect.y - ( templateRect.height >> 1 ),
                             templateRect.width << 1, templateRect.height << 1 );

            if ( 0 > templateRect.x || 0 > templateRect.y ||
                 img.cols < templateRect.x + templateRect.width ||
                 img.rows < templateRect.y + templateRect.height )
            {
                FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Invalid template rotate region";
                retVal = GC_ERR;
            }
            else
            {
                Mat matTemp = gray( rotateArea ).clone();
                Mat matTempRot = Mat::zeros( matTemp.size(), CV_8U );

                int templateCount = 2 * cvRound( SYMBOL_TEMPL_ANGLE_MAX / SYMBOL_TEMPL_ANGLE_INC ) + 1.0;
                int centerTemplateIdx = templateCount / 2;

                for ( int i = 0; i < templateCount; ++i )
                    templates.push_back( Mat::zeros( templ.size(), CV_8U ) );

                Rect roiRotate( templ.cols >> 1, templ.rows >> 1, templ.cols, templ.rows );
                Mat matTemplateTemp = matTemp( roiRotate );
                matTemplateTemp.copyTo( templates[ static_cast< size_t >( centerTemplateIdx ) ] );

                for ( int i = 0; i < centerTemplateIdx; ++i )
                {
                    retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i - centerTemplateIdx ) );
                    if ( GC_OK != retVal )
                        break;

                    matTemplateTemp = matTempRot( roiRotate );
                    matTemplateTemp.copyTo( templates[ static_cast< size_t >( i ) ] );

                    retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i + 1 ) );
                    if ( 0 != retVal )
                        break;

                    matTemplateTemp = matTempRot( roiRotate );
                    matTemplateTemp.copyTo( templates[ static_cast< size_t >( centerTemplateIdx + i + 1 ) ] );
                }
            }
        }
        if ( templates.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Could not create templates";
            retVal = GC_ERR;
        }
#ifdef jimbob // DEBUG_FIND_CALIB_SYMBOL
        else
        {
            for ( size_t i = 0; i < templates.size(); ++i )
            {
                imwrite( DEBUG_RESULT_FOLDER + to_string( i ) + ".png", templates[ i ] );
            }
        }
#endif
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;

}
GC_STATUS FindSymbol::Calibrate( const cv::Mat &img, const Scalar hsvRange_1_start, const Scalar hsvRange_1_end, const Scalar hsvRange_2_start, const Scalar hsvRange_2_end )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();

        cv::Mat1b mask;
        retVal = FindColorRange( img, mask, hsvRange_1_start, hsvRange_1_end, hsvRange_2_start, hsvRange_2_end );
        if ( GC_OK == retVal )
        {
            Mat kern = getStructuringElement( MORPH_ELLIPSE, Size( 3, 3 ) );
            dilate( mask, mask, kern, Point( -1, -1 ), 1 );
            erode( mask, mask, kern, Point( -1, -1 ), 3 );
            dilate( mask, mask, kern, Point( -1, -1 ), 5 );
            imwrite( "/var/tmp/water/yellow_triangles_mask.png", mask );

            model.clear();
            Mat color = img.clone();
            RotatedRect rotRect;
            vector< vector< Point > > contours;
            findContours( mask( Rect( 600, 110, 600, 400 ) ), contours, RETR_LIST, CHAIN_APPROX_SIMPLE );
            for ( size_t i = 0; i < contours.size(); ++i )
            {
                for ( size_t j = 0; j < contours[ i ].size(); ++j )
                {
                    contours[ i ][ j ].x += 600;
                    contours[ i ][ j ].y += 110;
                }
                rotRect = minAreaRect( contours[ i ] );
                // ellipse( color, rotRect, Scalar( 0, 255, 255 ) );

                cout << "i=" << i << " x=" << rotRect.center.x << " y=" << rotRect.center.y << endl;
                drawContours( color, contours, i, Scalar( 0, 255, 255 ), 3 );
                line( color, Point( rotRect.center.x - 7, rotRect.center.y ),
                              Point( rotRect.center.x + 7, rotRect.center.y ), Scalar( 0, 255, 0 ), 3 );
                line( color, Point( rotRect.center.x, rotRect.center.y - 7 ),
                              Point( rotRect.center.x, rotRect.center.y + 7 ), Scalar( 0, 255, 0 ), 3 );
            }

            // bottom right
            model.pixelPoints.push_back( Point2d( 1059.25, 374.75 ) );
            model.worldPoints.push_back( Point2d( 390.4, 196 - 55.5 ) );

            model.pixelPoints.push_back( Point2d( 800.25, 322.75 ) );
            model.worldPoints.push_back( Point2d( 116.8, 196 - 101.3 ) );

            model.pixelPoints.push_back( Point2d( 919, 270 ) );
            model.worldPoints.push_back( Point2d( 234.8, 196 - 10.0 ) );

            model.pixelPoints.push_back( Point2d( 1084, 226.5 ) );
            model.worldPoints.push_back( Point2d( 406.1, 196 - -88.4) );

            model.pixelPoints.push_back( Point2d( 699, 179 ) );
            model.worldPoints.push_back( Point2d( 0.0, 196 - 0.0 ) );

            imwrite( "/var/tmp/water/yellow_triangles.png", color );

            retVal = Calibrate( model.pixelPoints, model.worldPoints );
            if ( GC_OK == retVal )
            {
                retVal = DrawCalibration( img, color, true, false, false );
                if ( GC_OK == retVal )
                {
                    imwrite( "/var/tmp/water/yellow_triangle_calibration.png", color );
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::Calibrate( const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( pixelPts.empty() || worldPts.empty() || pixelPts.size() != worldPts.size() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::Calibrate] Invalid world and/or pixel point sets";
            retVal = GC_ERR;
        }
        else
        {
            matHomogPixToWorld = findHomography( pixelPts, worldPts );
            if ( matHomogPixToWorld.empty() )
            {
                FILE_LOG( logERROR ) << "[FindSymbol::Calibrate] Could not find pixel to world coordinate homography";
                retVal = GC_ERR;
            }
            else
            {
                matHomogWorldToPix = findHomography( worldPts, pixelPts );
                if ( matHomogPixToWorld.empty() )
                {
                    FILE_LOG( logERROR ) << "[FindSymbol::Calibrate] Could not find world to pixel coordinate homography";
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::FindColorRange( const cv::Mat &img, cv::Mat1b &mask,
                                      const cv::Scalar hsvRange_1_start, const cv::Scalar hsvRange_1_end,
                                      const cv::Scalar hsvRange_2_start, const cv::Scalar hsvRange_2_end )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindRed] Cannot find red in an empty image";
            retVal = GC_ERR;
        }
        else if ( img.type() != CV_8UC3 )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindRed] Image must be an 8-bit BGR image to find a color range";
            retVal = GC_ERR;
        }
        else
        {
            Mat3b hsv;
            cvtColor( img, hsv, COLOR_BGR2HSV );
            imwrite( "/var/tmp/water/hsv.png", hsv );

            if ( 0 > hsvRange_2_start.val[ 0 ] || 0 > hsvRange_2_end.val[ 0 ] )
            {
                inRange( hsv, hsvRange_1_start, hsvRange_1_end, mask );
            }
            else
            {
                Mat1b mask1, mask2;
                inRange( hsv, hsvRange_1_start, hsvRange_1_end, mask1 );
                inRange( hsv, hsvRange_2_start, hsvRange_2_end, mask2 );
                mask = mask1 | mask2;
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::FindRed] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::DrawCalibration( const cv::Mat &img, cv::Mat &result,
                                       const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogPixToWorld.empty() || matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::DrawCalibration] System not calibrated";
            retVal = GC_ERR;
        }
        else if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::DrawCalibration] Empty image";
            retVal = GC_ERR;
        }
        else
        {
            if ( CV_8UC1 == img.type() )
            {
                cvtColor( img, result, COLOR_GRAY2BGR );
            }
            else if ( CV_8UC3 == img.type() )
            {
                img.copyTo( result );
            }
            else
            {
                FILE_LOG( logERROR ) << "[FindSymbol::DrawCalibration] Invalid image type";
                retVal = GC_ERR;
            }

            if ( model.pixelPoints.empty() )
            {
                FILE_LOG( logERROR ) << "[FindSymbol::DrawCalibration] No symbol points to draw";
                retVal = GC_ERR;
            }
            else if ( GC_OK == retVal )
            {
                double dim = static_cast< double >( std::max( result.cols, result.rows ) );
                int lineWidth = max( 1, cvRound( dim / 300.0 ) ) >> 1;

                if ( drawCalib )
                {
                    line( result, Point( model.pixelPoints[ 0 ].x - lineWidth * 7, model.pixelPoints[ 0 ].y ),
                                  Point( model.pixelPoints[ 0 ].x + lineWidth * 7, model.pixelPoints[ 0 ].y ), Scalar( 0, 255, 0 ), lineWidth );
                    line( result, Point( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y - lineWidth * 7 ),
                                  Point( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y + lineWidth * 7 ), Scalar( 0, 255, 0 ), lineWidth );
                    for ( size_t i = 1; i < model.pixelPoints.size(); ++i )
                    {
                        // line( result, model.pixelPoints[ i - 1 ], model.pixelPoints[ i ], Scalar( 255, 0, 0 ), lineWidth );
                        line( result, Point( model.pixelPoints[ i ].x - lineWidth * 7, model.pixelPoints[ i ].y ),
                                      Point( model.pixelPoints[ i ].x + lineWidth * 7, model.pixelPoints[ i ].y ), Scalar( 0, 255, 0 ), lineWidth );
                        line( result, Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y - lineWidth * 7 ),
                                      Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y + lineWidth * 7 ), Scalar( 0, 255, 0 ), lineWidth );
                    }
                    // line( result, model.pixelPoints[ 0 ], model.pixelPoints[ model.pixelPoints.size() - 1 ], Scalar( 255, 0, 0 ), lineWidth );

#if 1
                    bool isFirst;
                    char msg[ 256 ];
                    Point2d pt1, pt2;
                    for ( double r = -5; r < 205; r += 20 )
                    {
                        isFirst = true;
                        for ( double c = -5; c < 405; c += 40 )
                        {
                            retVal = WorldToPixel( Point2d( c, r ), pt1 );
                            if ( GC_OK == retVal )
                            {
                                if ( isFirst )
                                {
                                    isFirst = false;
                                    sprintf( msg, "%.1f cm", r );
                                    putText( result, msg, Point( 560, pt1.y - 10 ), FONT_HERSHEY_PLAIN, static_cast< double >( lineWidth ) / 1.5, Scalar( 0, 200, 200 ), lineWidth );
                                }
                                retVal = WorldToPixel( Point2d( c + 41, r ), pt2 );
                                if ( GC_OK == retVal )
                                {
                                    line( result, pt1, pt2, Scalar( 0, 255, 255 ), 1 );
                                    retVal = WorldToPixel( Point2d( c, r + 39 ), pt2 );
                                    if ( GC_OK == retVal )
                                    {
                                        line( result, pt1, pt2, Scalar( 0, 255, 255 ), 1 );
                                    }
                                }
                            }
                        }
                    }
#else

                    Point2d ptLftTopPix( 0.0, 0.0 );
                    Point2d ptRgtTopPix( static_cast< double >( result.cols - 1 ), 0.0 );
                    Point2d ptLftBotPix( 0.0, static_cast< double >( result.rows - 1 ) );
                    Point2d ptRgtBotPix( static_cast< double >( result.cols - 1 ), static_cast< double >( result.rows - 1 ) );

                    Point2d ptLftTopW, ptRgtTopW, ptLftBotW, ptRgtBotW;
                    retVal = PixelToWorld( ptLftTopPix, ptLftTopW );
                    if ( GC_OK == retVal )
                    {
                        retVal = PixelToWorld( ptRgtTopPix, ptRgtTopW );
                        if ( GC_OK == retVal )
                        {
                            retVal = PixelToWorld( ptLftBotPix, ptLftBotW );
                            if ( GC_OK == retVal )
                            {
                                retVal = PixelToWorld( ptRgtBotPix, ptRgtBotW );
                                if ( GC_OK == retVal )
                                {
                                    double minXW = std::min( ptLftTopW.x, ptLftBotW.x );
                                    double maxXW = std::max( ptRgtTopW.x, ptRgtBotW.x );
                                    double minYW = std::min( ptLftTopW.y, ptRgtTopW.y );
                                    double maxYW = std::max( ptLftBotW.y, ptRgtBotW.y );

                                    double incX = ( maxXW - minXW ) / 10.0;
                                    double incY = ( maxYW - minYW ) / 10.0;

                                    bool isFirst;
                                    char msg[ 256 ];
                                    Point2d pt1, pt2;
                                    for ( double r = minYW; r > maxYW; r += incY )
                                    {
                                        isFirst = true;
                                        for ( double c = minXW; c < maxXW; c += incX )
                                        {
                                            retVal = WorldToPixel( Point2d( c, r ), pt1 );
                                            if ( GC_OK == retVal )
                                            {
                                                if ( isFirst )
                                                {
                                                    isFirst = false;
                                                    sprintf( msg, "%.1f cm", r );
                                                    putText( result, msg, Point( 10, pt1.y - 10 ), FONT_HERSHEY_PLAIN, static_cast< double >( lineWidth ) / 1.5, Scalar( 0, 0, 255 ), lineWidth );
                                                }
                                                retVal = WorldToPixel( Point2d( c + incX, r ), pt2 );
                                                if ( GC_OK == retVal )
                                                {
                                                    line( result, pt1, pt2, Scalar( 0, 255, 255 ), lineWidth );
                                                    retVal = WorldToPixel( Point2d( c, r + incY ), pt2 );
                                                    if ( GC_OK == retVal )
                                                    {
                                                        line( result, pt1, pt2, Scalar( 0, 255, 255 ), lineWidth );
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
#endif
                }
                if ( drawMoveROIs )
                {
                    rectangle( result, model.wholeTargetRegion, Scalar( 0, 0, 255 ), lineWidth );
                }
                if ( drawSearchROI )
                {

                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::DrawCalibration] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::RotateImage( const Mat &src, Mat &dst, const double angle )
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
        FILE_LOG( logERROR ) << "[FindSymbol::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindSymbol::FindTargets( const Mat &img, const cv::Rect targetRoi, const double minScore, const string resultFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat gray;
        if ( templates.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindTargets] Templates not defined";
            retVal = GC_ERR;
        }
        else if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindTargets] Cannot find targets in a NULL image";
            retVal = GC_ERR;
        }
        if ( 0.01 > minScore || 1.0 < minScore )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindTargets] Invalid minimum target score " << minScore;
            retVal = GC_ERR;
        }
        else if ( img.type() == CV_8UC3 )
        {
            cvtColor( img, gray, COLOR_BGR2GRAY );
        }
        else if ( img.type() == CV_8UC1 )
        {
            gray = img;
        }
        else
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Image must be a gray or BGR image to calibrate with a template match";
            retVal = GC_ERR;
        }
        if ( GC_OK == retVal )
        {
            retVal = MatchTemplate( templates.size() >> 1, gray, targetRoi, minScore, templates.size() * 2 );
            if ( GC_OK == retVal )
            {
                vector< TemplateFindItem > itemsTemp;
                for ( size_t i = 0; i < matchItems.size(); ++i )
                    itemsTemp.push_back( matchItems[ i ] );

                matchItems.clear();
                for ( size_t i = 0; i < itemsTemp.size(); ++i )
                {
                    for ( size_t j = 0; j < templates.size(); ++j )
                    {
                        retVal = MatchRefine( static_cast< int >( j ), gray, targetRoi, minScore, 1, itemsTemp[ i ] );
                        if ( GC_OK != retVal )
                            break;
                    }
                    if ( GC_OK != retVal )
                        break;
                    matchItems.push_back( itemsTemp[ i ] );
                }

                if ( !resultFilepath.empty() )
                {
                    Mat matTemp1;
                    cvtColor( gray, matTemp1, COLOR_GRAY2BGR );
                    for ( size_t i = 0; i < matchItems.size(); ++i )
                    {
                        line( matTemp1, Point( static_cast< int >( matchItems[ i ].pt.x ) - 5,
                                               static_cast< int >( matchItems[ i ].pt.y ) ),
                                Point( static_cast< int >( matchItems[ i ].pt.x ) + 5,
                                       static_cast< int >( matchItems[ i ].pt.y ) ), Scalar( 0, 0, 255 ) );
                        line( matTemp1, Point( static_cast< int >( matchItems[ i ].pt.x ),
                                               static_cast< int >( matchItems[ i ].pt.y ) - 5 ),
                                Point( static_cast< int >( matchItems[ i ].pt.x ),
                                       static_cast< int >( matchItems[ i ].pt.y ) + 5 ), Scalar( 0, 0, 255 ) );
                    }
                    bool isOK = imwrite( resultFilepath, matTemp1 );
                    if ( !isOK )
                    {
                        FILE_LOG( logERROR ) << "[" << __func__ << "[FindSymbol::FindTargets]"
                                                                   " Could not save result calib grid find to cache";
                        retVal = GC_ERR;
                    }
                }
#ifdef DEBUG_FIND_CALIB_SYMBOL   // output template matches to CSV file
                FILE *fp = fopen( ( DEBUG_RESULT_FOLDER + "matches.csv" ).c_str(), "w" );
                if ( nullptr != fp )
                {
                    fprintf( fp, "Score, X, Y\n" );
                    for ( size_t i = 0; i < matchItems.size(); ++i )
                    {
                        fprintf( fp, "%.3f, %.3f, %.3f\n", matchItems[ i ].score,
                                 matchItems[ i ].pt.x, matchItems[ i ].pt.y );
                    }
                    fclose( fp );
                }
#endif
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindSymbol::MatchRefine( const int index, const Mat &img, const Rect targetRoi,
                                   const double minScore, const int numToFind, TemplateFindItem &item )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 0 > index || static_cast< int >( templates.size() ) <= index )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchRefine]"
                                                       " Attempted to find template index=" << index << \
                                                       " Must be in range 0-" << templates.size() - 1;
            retVal = GC_ERR;
        }
        else if ( 0.05 > minScore || 1.0 < minScore )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchRefine]  Min score %.3f must be in range 0.05-1.0" << minScore;
            retVal = GC_ERR;
        }
        else if ( 1 > numToFind || 1000 < numToFind )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchRefine]"
                                                       " Attempted to find " << numToFind << \
                                                       " matches.  Must be in range 1-1000";
            retVal = GC_ERR;
        }
        else
        {
            Rect rect;
            Point2d ptFinal;
            Point ptMin, ptMax;
            double minScore, maxScore;
            TemplateFindItem itemTemp;

            Mat targetMat = img( targetRoi );
            int templateDimEvenCols = templates[ index ].cols + ( templates[ index ].cols % 2 );
            int templateDimEvenRows = templates[ index ].rows + ( templates[ index ].rows % 2 );
            Mat matchSpace = Mat::zeros( Size( targetRoi.width - templateDimEvenCols + 1,
                                               targetRoi.height - templateDimEvenRows + 1 ), CV_32F );

            matchTemplate( targetMat, templates[ static_cast< size_t >( index ) ], matchSpace, TM_CCOEFF_NORMED );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat matTemp;
            normalize( matchSpace, matTemp, 255.0 );
            imwrite( DEBUG_RESULT_FOLDER + "match_fine.png", matTemp );
#endif
            rect.x = std::max( 0, cvRound( item.pt.x )- ( templates[ 0 ].cols >> 1 ) - ( templates[ 0 ].cols >> 2 ) );
            rect.y = std::max( 0, cvRound( item.pt.y ) - ( templates[ 0 ].rows >> 1 ) - ( templates[ 0 ].rows >> 2 ) );
            rect.width = templates[ 0 ].cols + ( templates[ 0 ].cols >> 1 );
            rect.height = templates[ 0 ].rows + ( templates[ 0 ].rows >> 1 );
            if ( rect.x + rect.width >= targetMat.cols )
                rect.x = targetMat.cols - rect.width;
            if ( rect.y + rect.height >= targetMat.rows )
                rect.y = targetMat.rows - rect.height;

            Mat matROI = targetMat( rect );
            Mat matchSpaceSmall = Mat::zeros( Size( rect.width - templateDimEvenCols + 1,
                                                    rect.height - templateDimEvenRows + 1 ), CV_32F );

            matchTemplate( matROI, templates[ static_cast< size_t >( index ) ], matchSpaceSmall, TM_CCOEFF_NORMED );

            minMaxLoc( matchSpaceSmall, &minScore, &maxScore, &ptMin, &ptMax );
            if ( maxScore > item.score )
            {
                retVal = SubpixelPointRefine( matchSpaceSmall, ptMax, ptFinal );
                if ( GC_OK == retVal )
                {
                    // ptFinal = Point2d( static_cast< double >( ptMax.x ), static_cast< double >( ptMax.y ) );
                    item.score = maxScore;
                    item.pt.x = static_cast< double >( rect.x + targetRoi.x ) + ptFinal.x + static_cast< double >( templates[ 0 ].cols ) / 2.0;
                    item.pt.y = static_cast< double >( rect.y + targetRoi.y ) + ptFinal.y + static_cast< double >( templates[ 0 ].rows ) / 2.0;
                }
                else
                {
                    item.score = 0.0;
                    item.pt.x = static_cast< double >( rect.x + targetRoi.x ) + ptMax.x + static_cast< double >( templates[ 0 ].cols ) / 2.0;
                    item.pt.y = static_cast< double >( rect.y + targetRoi.y ) + ptMax.y + static_cast< double >( templates[ 0 ].rows ) / 2.0;
                    retVal = GC_OK;
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::MatchRefine] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindSymbol::MatchTemplate( const int index, const Mat &img, const Rect targetRoi, const double minScore, const int numToFind )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( 0 > index || static_cast< int >( templates.size() ) <= index )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchTemplate]"
                                    " Attempted to find template index=" << index << \
                                    " Must be in range 0-" << templates.size() - 1;
            retVal = GC_ERR;
        }
        else if ( 0.05 > minScore || 1.0 < minScore )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchTemplate]"
                                    " Min score %.3f must be in range 0.05-1.0" << minScore;
            retVal = GC_ERR;
        }
        else if ( 1 > numToFind || 1000 < numToFind )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::MatchTemplate]"
                                                       " Attempted to find " << numToFind << \
                                                       " matches.  Must be in range 1-1000";
            retVal = GC_ERR;
        }
        else if ( img.type() != CV_8UC1 )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::CreateTemplates] Image must be n gray image to calibrate with a template match";
            retVal = GC_ERR;
        }

        if ( GC_OK == retVal )
        {
            Rect rect;
            double dMin, dMax;
            Point ptMin, ptMax;
            Point2d ptFinal;

            Mat matchSpace;
            TemplateFindItem itemTemp;

            matchItems.clear();
            matchTemplate( img( targetRoi ), templates[ static_cast< size_t >( index ) ], matchSpace, cv::TM_CCOEFF_NORMED );

#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat matTemp;
            normalize( matchSpace, matTemp, 255.0 );
            imwrite( DEBUG_RESULT_FOLDER + "match_original.png", img );
            imwrite( DEBUG_RESULT_FOLDER + "match_coarse.png", matTemp );
            imwrite( DEBUG_RESULT_FOLDER + "match_coarse_double.tiff", matchSpace );
#endif

            for ( int i = 0; i < numToFind; i++ )
            {
                minMaxLoc( matchSpace, &dMin, &dMax, &ptMin, &ptMax );
                if (  0 < ptMax.x && 0 < ptMax.y && img.cols - 1 > ptMax.x && img.rows - 1 > ptMax.y )
                {
                    if ( dMax >= minScore )
                    {
                        itemTemp.score = dMax;
                        itemTemp.pt.x = static_cast< double >( ptMax.x + targetRoi.x ) + static_cast< double >( templates[ 0 ].cols ) / 2.0;
                        itemTemp.pt.y = static_cast< double >( ptMax.y + targetRoi.y ) + static_cast< double >( templates[ 0 ].rows ) / 2.0;
                        matchItems.push_back( itemTemp );
                    }
                    else
                        break;
                }
                circle( matchSpace, ptMax, 17, Scalar( 0.0 ), FILLED );
            }
            if ( matchItems.empty() )
            {
                FILE_LOG( logERROR ) << "[FindSymbol::MatchTemplate] No template matches found";
                retVal = GC_ERR;
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::MatchTemplate] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindSymbol::SubpixelPointRefine( const Mat &matchSpace, const Point ptMax, Point2d &ptResult )
{
    GC_STATUS retVal = GC_OK;
    if ( 1 > ptMax.x || 1 > ptMax.y || matchSpace.cols - 2 < ptMax.x || matchSpace.rows - 2 < ptMax.y )
    {
#ifdef DEBUG_FIND_CALIB_GRID
        FILE_LOG( logWARNING ) << "[FindSymbol::SubpixelPointRefine] Invalid point (not on image) for subpixel refinement";
#endif
        retVal = GC_WARN;
    }
    else if ( CV_32FC1 != matchSpace.type() )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::SubpixelPointRefine] Invalid image format for subpixel refinement";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            float val;
            int col, row;
            float total = 0.0f;
            float totalX = 0.0f;
            float totalY = 0.0f;
            for ( row = ptMax.y - 1; row < ptMax.y + 2; ++row )
            {
                for ( col = ptMax.x - 1; col < ptMax.x + 2; ++col )
                {
                    val = matchSpace.at< float >( Point( col, row ) );
                    total += val;
                    totalX += val * static_cast< float >( col );
                    totalY += val * static_cast< float >( row );
                }
            }
            ptResult = Point2d( static_cast< double >( totalX / total ),
                                static_cast< double >( totalY / total ) );
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::SubpixelPointRefine] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}

} // namespace gc
