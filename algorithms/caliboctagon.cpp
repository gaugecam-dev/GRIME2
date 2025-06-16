#include "log.h"
#include "caliboctagon.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/foreach.hpp>
#include <filesystem>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <iterator>
#include "searchlines.h"

#ifdef DEBUG_FIND_CALIB_SYMBOL
#undef DEBUG_FIND_CALIB_SYMBOL
#include <iostream>
#ifdef _WIN32
static const char DEBUG_FOLDER[] = "c:/gaugecam/";
#else
static const string DEBUG_FOLDER = string( "/var/tmp/gaugecam/" );
#endif
#endif

static const double MIN_SYMBOL_CONTOUR_SIZE = 8;
using namespace cv;
using namespace std;
using namespace boost;
namespace fs = std::filesystem;

namespace gc
{

// static double elongation( Moments m );
static double distance( Point2d a, Point2d b );

CalibOctagon::CalibOctagon()
{
    try
    {
        moveRefLftPt = cv::Point2d( -1.0, -1.0 );
        moveRefRgtPt = cv::Point2d( -1.0, -1.0 );
#ifdef _WIN32
        if ( !fs::exists( CACHE_FOLDER ) )
        {
            fs::create_directories( CACHE_FOLDER );
        }
#else
        if ( !fs::exists( CACHE_FOLDER ) )
        {
            fs::create_directories( CACHE_FOLDER );
        }
#endif
#ifdef DEBUG_FIND_CALIB_SYMBOL
        if ( !fs::exists( DEBUG_FOLDER ) )
        {
            bool bRet = fs::create_directories( DEBUG_FOLDER );
            if ( !bRet )
            {
                FILE_LOG( logWARNING ) << "[CalibOctagon::CalibOctagon] Could not create debug folder " << DEBUG_FOLDER;
            }
        }
#endif
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagon] Creating debug folder" << diagnostic_information( e );
    }
}
void CalibOctagon::clear()
{
    matHomogPixToWorld = Mat();
    matHomogWorldToPix = Mat();
    model.clear();
}
GC_STATUS CalibOctagon::GetCalibParams( std::string &calibParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        stringstream ss;
        ss.setf( ios::fixed, ios::floatfield );
        ss.precision( 3 );
        ss << "STOP SIGN CALIBRATION" << endl;
        ss << "Association points" << endl;
        for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
        {
            ss << "pixel x=" << model.pixelPoints[ i ].x << " y=" << model.pixelPoints[ i ].y;
            ss << "  world x=" << model.worldPoints[ i ].x << " y=" << model.worldPoints[ i ].y << endl;
        }
        calibParams = ss.str();
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::GetCalibParams] " << e.what();
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibOctagon::DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, string &err_msg )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        err_msg.clear();
        if ( img.empty() )
        {
            err_msg = "[CalibOctagon::DrawAssocPts] Needs non-empty input images";
            FILE_LOG( logERROR ) << "[CalibOctagon::DrawAssocPts] Needs non-empty input images";
            retVal = GC_ERR;
        }
        else
        {
            if ( CV_8UC1 == img.type() )
            {
                cvtColor( img, overlay, COLOR_GRAY2BGR );
            }
            else
            {
                img.copyTo( overlay );
            }
            char buf[ 256 ];
            for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
            {
                // circle( overlay, model.pixelPoints[ i ], 3, Scalar( 0, 255, 0 ), FILLED );
                Point textStart = Point( model.pixelPoints[ i ].x - 10, model.pixelPoints[ i ].y - 50 );
                overlay( Rect( textStart.x - 5, textStart.y - 15, 120, 50 ) ) = Scalar( 255, 255, 255 );
                sprintf( buf, "%d p:x=%d y=%d", static_cast< int >( i ), cvRound( model.pixelPoints[ i ].x ), cvRound( model.pixelPoints[ i ].y ) );
                putText( overlay, buf, textStart, FONT_HERSHEY_PLAIN, 0.8, Scalar( 0, 0, 0 ), 1 );

                textStart.y += 25;
                sprintf( buf, "w:x=%.1f y=%.1f", model.worldPoints[ i ].x, model.worldPoints[ i ].y );
                putText( overlay, buf, textStart, FONT_HERSHEY_PLAIN, 0.8, Scalar( 0, 0, 0 ), 1 );
            }
        }
    }
    catch( const Exception &e )
    {
        err_msg = "[CalibOctagon::DrawAssocPts] EXCEPTION";
        FILE_LOG( logERROR ) << "[CalibOctagon::DrawAssocPts] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// symbolPoints are clockwise ordered with 0 being the topmost left point
GC_STATUS CalibOctagon::Calibrate( const cv::Mat &img, const std::string &controlJson, string &err_msg )
{
    GC_STATUS retVal = GC_OK;

    CalibModelOctagon oldModel;
    cv::Mat oldHomogPixToWorld, oldHomogWorldToPix;
    try
    {
        // cout << model.targetSearchRegion << endl;

        // copy previous model
        matHomogPixToWorld.copyTo( oldHomogPixToWorld );
        matHomogWorldToPix.copyTo( oldHomogWorldToPix );
        oldModel = model;

        bool useRoi = -1 != model.targetSearchRegion.x ||
                -1 != model.targetSearchRegion.y ||
                -1 != model.targetSearchRegion.width ||
                -1 != model.targetSearchRegion.height;

        Mat scratch = img;
        if ( useRoi )
        {
            scratch = img( model.targetSearchRegion );
        }
        retVal = octagonSearch.Find( scratch, model.pixelPoints, true );
        if ( GC_OK == retVal )
        {
            retVal = TestCalibration( model.validCalib );
        }
        if ( GC_OK != retVal )
        {
            retVal = octagonSearch.Find( scratch, model.pixelPoints, false );
            if ( GC_OK == retVal )
            {
                retVal = TestCalibration( model.validCalib );
            }
        }
        if ( GC_OK != retVal )
        {
            retVal = octagonSearch.FindScale( scratch, model.pixelPoints, 2.0 );
            if ( GC_OK == retVal )
            {
                retVal = TestCalibration( model.validCalib );
            }
        }
        if ( GC_OK != retVal )
        {
            err_msg = "CALIB FAIL [octagon] Could not find octagon in image";
        }
        else
        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            if ( useRoi )
            {
                img( model.targetSearchRegion ).copyTo( color );
            }
            else
            {
                img.copyTo( color );
            }
            for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
            {
                line( color, Point( model.pixelPoints[ i ].x - 10, model.pixelPoints[ i ].y ),
                             Point( model.pixelPoints[ i ].x + 10, model.pixelPoints[ i ].y ),
                      Scalar( 0, 255, 255 ), 1 );
                line( color, Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y - 10 ),
                             Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y + 10 ),
                      Scalar( 0, 255, 255 ), 1 );
            }
            imwrite( DEBUG_FOLDER + "___FINAL.png", color );
#endif
            if ( useRoi )
            {
                Point2d offset = Point2d( model.targetSearchRegion.x, model.targetSearchRegion.y );
                for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
                {
                    model.pixelPoints[ i ] += offset;
                }
            }

            retVal = CalcOctoWorldPoints( model.facetLength, model.worldPoints );
            if ( GC_OK != retVal )
            {
                err_msg = "CALIB FAIL [octagon] Could not calculate octogan points";
            }
            else
            {
                vector< Point2d > pointsTemp;
                Point2d ptTemp( 0.0, model.zeroOffset );
                for ( size_t i = 0; i < model.worldPoints.size(); ++i )
                {
                    pointsTemp.push_back( model.worldPoints[ i ] + ptTemp );
                }
                retVal = CreateCalibration( model.pixelPoints, pointsTemp );
                if ( GC_OK != retVal )
                {
                    err_msg = "CALIB FAIL [octagon] Could not create calibaration";
                }
                else
                {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                    Mat temp_img;
                    img.copyTo( temp_img );
                    // cvtColor( img, temp_img, COLOR_GRAY2BGR );
                    line( temp_img, model.waterlineSearchCorners[ 0 ], model.waterlineSearchCorners[ 1 ], Scalar( 0, 0, 255 ), 7 );
                    line( temp_img, model.waterlineSearchCorners[ 0 ], model.waterlineSearchCorners[ 2 ], Scalar( 0, 255, 255 ), 7 );
                    line( temp_img, model.waterlineSearchCorners[ 3 ], model.waterlineSearchCorners[ 1 ], Scalar( 0, 255, 0 ), 7 );
                    line( temp_img, model.waterlineSearchCorners[ 3 ], model.waterlineSearchCorners[ 2 ], Scalar( 255, 0, 0 ), 7 );
                    imwrite( "/var/tmp/gaugecam/test_search_roi_0.png", temp_img );
                    Mat color;
                    img.copyTo( color );
                    circle( color, model.waterlineSearchCorners[ 0 ], 11, Scalar( 0, 0, 255 ), 3 );
                    circle( color, model.waterlineSearchCorners[ 1 ], 11, Scalar( 0, 255, 255 ), 3 );
                    circle( color, model.waterlineSearchCorners[ 2 ], 11, Scalar( 255, 0, 0 ), 3 );
                    circle( color, model.waterlineSearchCorners[ 3 ], 11, Scalar( 0, 255, 0 ), 3 );
                    imwrite( "/var/tmp/gaugecam/roi_pts.png", color );
#endif
                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // KWC ~~~ this is where to start the waterline roi offset problem search
                    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    // int x_offset = std::round( oldModel.center.x - model.center.x );
                    // int y_offset = std::round( oldModel.center.y - model.center.y );
                    // for ( size_t i = 0; i < model.waterlineSearchCorners.size(); ++i )
                    // {
                    //     model.waterlineSearchCorners[ i ].x += x_offset;
                    //     model.waterlineSearchCorners[ i ].y += y_offset;
                    // }

                    SearchLines searchLines;
                    retVal = searchLines.CalcSearchLines( model.waterlineSearchCorners, model.searchLineSet );
                    if ( GC_OK != retVal )
                    {
                        err_msg = "CALIB FAIL [octagon] Invalid search lines (is 4-pt bounding poly correct?)";
                        FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] Invalid search lines (is 4-pt bounding poly correct?)";
                        retVal = GC_OK;
                    }
                    else
                    {
                        // offset_x = model.oldPixelPoints[ 4 ].x - model.pixelPoints[ 4 ].x;
                        // offset_y = model.oldPixelPoints[ 4 ].y - model.pixelPoints[ 4 ].y;
                        retVal = CalcCenterAngle( model.worldPoints, model.center, model.angle );
                        if ( GC_OK != retVal )
                        {
                            err_msg = "CALIB FAIL [octagon] Could not octagon angle";
                        }
                        else
                        {
                            model.imgSize = img.size();
                        }
                    }
                }
            }
            if ( model.pixelPoints.empty() || model.worldPoints.empty() || model.searchLineSet.empty() )
            {
                err_msg = "CALIB FAIL [octagon] No valid calibration for drawing";
                FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] No valid calibration for drawing";
                retVal = GC_ERR;
            }
            else if ( matHomogPixToWorld.empty() || matHomogWorldToPix.empty() )
            {
                err_msg = "CALIB FAIL [octagon] System not calibrated";
                FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] System not calibrated";
                retVal = GC_ERR;
            }
            else
            {
                model.controlJson = controlJson;
            }
        }
        if ( GC_OK == retVal )
        {
            if ( model.oldPixelPoints.empty() )
            {
                model.oldPixelPoints = model.pixelPoints;
            }
            // if ( !noSave )
            // {
            //     moveRefLftPt = model.pixelPoints[ 5 ];
            //     moveRefRgtPt = model.pixelPoints[ 4 ];
            // }
        }
        else
        {
            model.validCalib = false;
        }
    }
    catch( cv::Exception &e )
    {
        err_msg = "CALIB FAIL [stop sign] Exception";
        FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    if ( GC_OK != retVal )
    {
        oldHomogPixToWorld.copyTo( matHomogPixToWorld );
        oldHomogWorldToPix.copyTo( matHomogWorldToPix );
        model = oldModel;
    }

    return retVal;
}
GC_STATUS CalibOctagon::MoveRefPoint(  cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt, const bool force )
{
    GC_STATUS retVal = GC_OK;
    if ( force || ( 0.0 >= moveRefLftPt.x ||  0.0 >= moveRefLftPt.y ||
                  ( 0.0 >= moveRefRgtPt.x ||  0.0 >= moveRefRgtPt.y ) ) )
    {
        if ( 8 != model.oldPixelPoints.size() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::MoveRefPoint] Cannot retrieve move reference point from an uncalibrated system";
            retVal = GC_ERR;
        }
        else
        {
            moveRefLftPt = model.oldPixelPoints[ 5 ];
            moveRefRgtPt = model.oldPixelPoints[ 4 ];
        }
    }
    lftRefPt = moveRefLftPt;
    rgtRefPt = moveRefRgtPt;
    return retVal;
}
GC_STATUS CalibOctagon::AdjustOctagonForRotation( const Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double octagonAngle = atan2( ( model.pixelPoints[ 4 ].y - model.pixelPoints[ 5 ].y ),
                                      ( model.pixelPoints[ 4 ].x - model.pixelPoints[ 5 ].x ) ) * ( 180.0 / CV_PI );
        double waterLineAngle = atan2( ( calcLinePts.lftPixel.y - calcLinePts.rgtPixel.y ),
                                       ( calcLinePts.lftPixel.x - calcLinePts.rgtPixel.x ) ) * ( 180.0 / CV_PI );
        if ( 90.0 < octagonAngle )
            octagonAngle += -180.0;
        else if ( -90.0 > octagonAngle )
            octagonAngle += 180.0;
        if ( 90.0 < waterLineAngle )
            waterLineAngle += -180.0;
        else if ( -90.0 > waterLineAngle )
            waterLineAngle += 180.0;

        // cout << "Stop sign angle=" << octagonAngle << " Waterline angle=" << waterLineAngle << endl;

        offsetAngle = octagonAngle - waterLineAngle;

        Mat mask = Mat::zeros( imgSize, CV_8UC1 );

        vector< Point > contour;
        for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
        {
            contour.push_back( model.pixelPoints[ i ] );
        }
        contour.push_back( model.pixelPoints[ 0 ] );


        drawContours( mask, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), FILLED );
        // circle( mask, model.pixelPoints[ 5 ], 13, Scalar( 128 ), 3 );
        // line( mask, calcLinePts.lftPixel, calcLinePts.rgtPixel, Scalar( 255 ), 5 );
        // putText( mask, "ORIGINAL", Point( model.pixelPoints[ 7 ].x, model.pixelPoints[ 0 ].y - 70 ), FONT_HERSHEY_PLAIN, 3.0, Scalar( 255 ), 3 );
        // imwrite( "/var/tmp/gaugecam/octagon_angle_original.png", mask );
        // rectangle( mask, Rect( model.pixelPoints[ 7 ].x - 10, 0, 500, model.pixelPoints[ 0 ].y - 10 ), Scalar( 0 ), FILLED );

        // line( mask, calcLinePts.lftPixel, calcLinePts.rgtPixel, Scalar( 0 ), 7 );

//        imwrite( "/var/tmp/gaugecam/octagon_angle_pre_adjusted.png", mask );
        Mat rotMatrix = cv::getRotationMatrix2D( model.pixelPoints[ 5 ], offsetAngle, 1.0 );
        warpAffine( mask, mask, rotMatrix, mask.size() );

//        line( mask, calcLinePts.lftPixel, calcLinePts.rgtPixel, Scalar( 255 ), 5 );
//        putText( mask, "ADJUSTED", Point( model.pixelPoints[ 7 ].x, model.pixelPoints[ 0 ].y - 70 ), FONT_HERSHEY_PLAIN, 3.0, Scalar( 255 ), 3 );

//        imwrite( "/var/tmp/gaugecam/octagon_angle_adjusted.png", mask );

        vector< vector< Point > > contours;
        findContours( mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

        if ( contours.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::AdjustOctagonForRotation] Could not find rotate adjusted stop sign";
            retVal = GC_ERR;
        }
        else
        {
            OctagonLines octoLines;
            retVal = FindCorners( mask, contours[ 0 ], octoLines );
            if ( GC_OK == retVal )
            {
                retVal = FindDiagonals( mask, contours[ 0 ], octoLines );
                if ( GC_OK == retVal )
                {
                    vector< Point2d > adjustedCorners;
                    retVal = CalcCorners( octoLines, adjustedCorners );
                    if ( GC_OK == retVal )
                    {
                        retVal = CalcHomographies();
                    }
                }
            }
        }
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::AdjustOctagonForRotation] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibOctagon::CalcCenterAngle( const std::vector< cv::Point2d > &pts, cv::Point2d &center, double &angle )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( pts.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] Empty point sets";
            retVal = GC_ERR;
        }
        else
        {
            center = Point2d( 0.0, 0.0 );
            for ( size_t i = 0; i < pts.size(); ++i )
            {
                center.x += pts[ i ].x;
                center.y += pts[ i ].y;
            }
            center.x /= static_cast< double >( pts.size() );
            center.y /= static_cast< double >( pts.size() );

            vector< Point2d > ptsSortY = pts;
            sort( ptsSortY.begin(), ptsSortY.end(), []( Point2d const &a, Point2d const &b ) { return ( a.y < b.y ); } );
            Point2d ptLftTop = ptsSortY[ ptsSortY[ 0 ].x < ptsSortY[ 1 ].x ? 0 : 1 ];
            Point2d ptRgtTop = ptsSortY[ ptsSortY[ 0 ].x < ptsSortY[ 1 ].x ? 1 : 0 ];

            angle = atan2( ptRgtTop.y - ptLftTop.y, ptRgtTop.x - ptLftTop.x ) * ( 180.0 / CV_PI );
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcCenterAngle] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibOctagon::CalcHomographies()
{
    GC_STATUS retVal = GC_OK;
    try
    {
        vector< Point2d > offsetWorldPts;
        Point2d offsetPt( 0.0, model.zeroOffset );
        for ( size_t i = 0; i < model.worldPoints.size(); ++i )
        {
            offsetWorldPts.push_back( model.worldPoints[ i ] + offsetPt );
        }
        retVal = CreateCalibration( model.pixelPoints, offsetWorldPts );
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcHomographies] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibOctagon::CreateCalibration( const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        model.validCalib = false;
        bool arePtsOk;
        retVal = TestCalibration( arePtsOk );
        if ( GC_OK == retVal && arePtsOk )
        {
            matHomogPixToWorld = findHomography( pixelPts, worldPts );
            if ( matHomogPixToWorld.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] Could not find pixel to world coordinate homography";
                retVal = GC_ERR;
            }
            else
            {
                matHomogWorldToPix = findHomography( worldPts, pixelPts );
                if ( matHomogPixToWorld.empty() )
                {
                    FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] Could not find world to pixel coordinate homography";
                    retVal = GC_ERR;
                }
                else
                {
                    model.validCalib = true;
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::SetCalibModel( CalibModelOctagon newModel )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        model = newModel;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::SetCalibModel] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::Load( const std::string jsonCalString )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( jsonCalString.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::Load] Bow tie calibration string is empty";
            retVal = GC_ERR;
        }
        else
        {
            stringstream ss;
            ss << jsonCalString;
            // cout << endl << endl << ss.str() << endl;
            property_tree::ptree ptreeTop;
            property_tree::json_parser::read_json( ss, ptreeTop );

            model.clear();
            model.imgSize.width = ptreeTop.get< int >( "imageWidth", 0 );
            model.imgSize.height = ptreeTop.get< int >( "imageHeight", 0 );
            model.facetLength = ptreeTop.get< double >( "facetLength", -1.0 );
            model.zeroOffset = ptreeTop.get< double >( "zeroOffset", 0.0 );

            Point2d ptTemp;
            property_tree::ptree::const_assoc_iterator it = ptreeTop.find( "PixelToWorld" );
            if ( it != ptreeTop.not_found() )
            {
                property_tree::ptree ptreeCalib = ptreeTop.get_child( "PixelToWorld" );
                BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeCalib.get_child( "points" ) )
                {
                    ptTemp.x = node.second.get< double >( "pixelX", 0.0 );
                    ptTemp.y = node.second.get< double >( "pixelY", 0.0 );
                    model.pixelPoints.push_back( ptTemp );
                    ptTemp.x = node.second.get< double >( "worldX", 0.0 );
                    ptTemp.y = node.second.get< double >( "worldY", 0.0 );
                    model.worldPoints.push_back( ptTemp );
                }
            }

            it = ptreeTop.find( "TargetSearchRegion" );
            if ( it != ptreeTop.not_found() )
            {
                const property_tree::ptree &ptreeTargetSearchROIs = ptreeTop.get_child( "TargetSearchRegion" );
                model.targetSearchRegion.x =      ptreeTargetSearchROIs.get< int >( "x", 0 );
                model.targetSearchRegion.y =      ptreeTargetSearchROIs.get< int >( "y", 0 );
                model.targetSearchRegion.width =  ptreeTargetSearchROIs.get< int >( "width", 0 );
                model.targetSearchRegion.height = ptreeTargetSearchROIs.get< int >( "height", 0 );
            }

            Point ptInt;
            it = ptreeTop.find( "WaterlineSearchRegion" );
            if ( it != ptreeTop.not_found() )
            {
                const property_tree::ptree &ptreeWaterlineSearchROI = ptreeTop.get_child( "WaterlineSearchRegion" );
                ptInt.x = ptreeWaterlineSearchROI.get< int >( "toplft_x", -1 );
                ptInt.y = ptreeWaterlineSearchROI.get< int >( "toplft_y", -1 );
                model.waterlineSearchCorners.push_back( ptInt );
                ptInt.x = ptreeWaterlineSearchROI.get< int >( "toprgt_x", -1 );
                ptInt.y = ptreeWaterlineSearchROI.get< int >( "toprgt_y", -1 );
                model.waterlineSearchCorners.push_back( ptInt );
                ptInt.x = ptreeWaterlineSearchROI.get< int >( "botlft_x", -1 );
                ptInt.y = ptreeWaterlineSearchROI.get< int >( "botlft_y", -1 );
                model.waterlineSearchCorners.push_back( ptInt );
                ptInt.x = ptreeWaterlineSearchROI.get< int >( "botrgt_x", -1 );
                ptInt.y = ptreeWaterlineSearchROI.get< int >( "botrgt_y", -1 );
                model.waterlineSearchCorners.push_back( ptInt );
            }

            Point ptTop, ptBot;
            it = ptreeTop.find( "SearchLines" );
            if ( it != ptreeTop.not_found() )
            {
                BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeTop.get_child( "SearchLines" ) )
                {
                    ptTop.x = node.second.get< int >( "topX", std::numeric_limits< int >::min() );
                    ptTop.y = node.second.get< int >( "topY", std::numeric_limits< int >::min() );
                    ptBot.x = node.second.get< int >( "botX", std::numeric_limits< int >::min() );
                    ptBot.y = node.second.get< int >( "botY", std::numeric_limits< int >::min() );
                    model.searchLineSet.push_back( LineEnds( ptTop, ptBot ) );
                }
            }

#ifdef LOG_CALIB_VALUES
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Camera calibration association points";
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Columns=" << cols << " Rows=" << rows;
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
#endif
            if ( 5 > model.pixelPoints.size() )
            {
                FILE_LOG( logERROR ) << "[CalibOctagon::Load] Invalid association point count";
                retVal = GC_ERR;
            }
            else
            {
#ifdef LOG_CALIB_VALUES
                for ( size_t i = 0; i < m_settings.pixelPoints.size(); ++i )
                {
                    FILE_LOG( logINFO ) << "[r=" << i / cols << " c=" << i % cols << "] " << \
                                           " pixelX=" << m_settings.pixelPoints[ i ].x << " pixelY=" << m_settings.pixelPoints[ i ].y << \
                                           " worldX=" << m_settings.worldPoints[ i ].x << " worldY=" << m_settings.worldPoints[ i ].y;
                }
#endif

                model.controlJson = ptreeTop.get< string >( "control_json", "{}" );
                model.oldPixelPoints = model.pixelPoints;
                retVal = CalcHomographies();
            }
#ifdef LOG_CALIB_VALUES
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            FILE_LOG( logINFO ) << "Search lines";
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            for ( size_t i = 0; i < m_settings.searchLines.size(); ++i )
            {
                FILE_LOG( logINFO ) << "[index=" << i << "] " << \
                                       " topX=" << m_settings.searchLines[ i ].top.x << " topY=" << m_settings.searchLines[ i ].top.y << \
                                       " botX=" << m_settings.searchLines[ i ].bot.x << " botY=" << m_settings.searchLines[ i ].bot.y;
            }
#endif
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::Save( const std::string jsonCalFilepath )
{
    GC_STATUS retVal = GC_OK;

    if ( model.pixelPoints.empty() || model.worldPoints.empty() ||
         model.pixelPoints.size() != model.worldPoints.size() || model.searchLineSet.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::Save] Empty cal point vector(s). Saves not possible without a calibrated object";
        retVal = GC_ERR;
    }
    else if ( jsonCalFilepath.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::Save] Calibration filepath is empty";
        retVal = GC_ERR;
    }
    else if ( string::npos == jsonCalFilepath.find( ".json" ) )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::Save] Filename must have .json extension";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            ofstream fileStream( jsonCalFilepath );
            if ( fileStream.is_open() )
            {
                fileStream << "{" << endl;
                fileStream << "  \"calibType\":\"Octagon\"" << "," << endl;
                fileStream << "  \"imageWidth\":" << model.imgSize.width << "," << endl;
                fileStream << "  \"imageHeight\":" << model.imgSize.height << "," << endl;
                fileStream << "  \"facetLength\":" << model.facetLength << "," << endl;
                fileStream << "  \"zeroOffset\":" << model.zeroOffset << "," << endl;

                fileStream << "  \"PixelToWorld\": " << endl;
                fileStream << "  {" << endl;
                fileStream << "    \"points\": [" << endl;
                fileStream << fixed << setprecision( 3 );
                for ( size_t i = 0; i < model.pixelPoints.size() - 1; ++i )
                {
                    fileStream << "      { \"pixelX\": " << model.pixelPoints[ i ].x << ", " << \
                                          "\"pixelY\": " << model.pixelPoints[ i ].y << ", " << \
                                          "\"worldX\": " << model.worldPoints[ i ].x << ", " << \
                                          "\"worldY\": " << model.worldPoints[ i ].y << " }," << endl;
                }
                fileStream << "      { \"pixelX\": " << model.pixelPoints[ model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"pixelY\": " << model.pixelPoints[ model.pixelPoints.size() - 1 ].y << ", " << \
                                      "\"worldX\": " << model.worldPoints[ model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"worldY\": " << model.worldPoints[ model.pixelPoints.size() - 1 ].y << " }" << endl;
                fileStream << "    ]" << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"TargetSearchRegion\": " << endl;
                fileStream << "  {" << endl;
                fileStream << fixed << setprecision( 0 );
                fileStream << "      \"x\": " <<      model.targetSearchRegion.x << ", " << \
                                    "\"y\": " <<      model.targetSearchRegion.y << ", " << \
                                    "\"width\": " <<  model.targetSearchRegion.width << ", " << \
                                    "\"height\": " << model.targetSearchRegion.height << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"WaterlineSearchRegion\": " << endl;
                fileStream << "  {" << endl;
                fileStream << fixed << setprecision( 0 );
                fileStream << "      \"toplft_x\": " << model.waterlineSearchCorners[ 0 ].x << ", " << \
                                    "\"toplft_y\": " << model.waterlineSearchCorners[ 0 ].y << ", " << \
                                    "\"toprgt_x\": " << model.waterlineSearchCorners[ 1 ].x << ", " << \
                                    "\"toprgt_y\": " << model.waterlineSearchCorners[ 1 ].y << ", " << \
                                    "\"botlft_x\": " << model.waterlineSearchCorners[ 2 ].x << ", " << \
                                    "\"botlft_y\": " << model.waterlineSearchCorners[ 2 ].y << ", " << \
                                    "\"botrgt_x\": " << model.waterlineSearchCorners[ 3 ].x << ", " << \
                                    "\"botrgt_y\": " << model.waterlineSearchCorners[ 3 ].y << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"SearchLines\": [" << endl;
                for ( size_t i = 0; i < model.searchLineSet.size() - 1; ++i )
                {
                    fileStream << "      { \"topX\": " << model.searchLineSet[ i ].top.x << ", " << \
                                          "\"topY\": " << model.searchLineSet[ i ].top.y << ", " << \
                                          "\"botX\": " << model.searchLineSet[ i ].bot.x << ", " << \
                                          "\"botY\": " << model.searchLineSet[ i ].bot.y << " }," << endl;
                }
                fileStream << "      { \"topX\": " << model.searchLineSet[ model.searchLineSet.size() - 1 ].top.x << ", " << \
                                      "\"topY\": " << model.searchLineSet[ model.searchLineSet.size() - 1 ].top.y << ", " << \
                                      "\"botX\": " << model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.x << ", " << \
                                      "\"botY\": " << model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.y << " }" << endl;
                fileStream << "  ]," << endl;

                string escaped;
                for ( size_t i = 0; i < model.controlJson.size(); ++i )
                {
                    if ( model.controlJson[ i ] == '\"' )
                        escaped += "\\\"";
                    else
                        escaped += model.controlJson[ i ];
                }

                fileStream << "  \"control_json\": \"" << escaped << "\"" << endl;
                fileStream << "}" << endl;
                fileStream.close();
            }
            else
            {
                FILE_LOG( logERROR ) << "[CalibOctagon::Save]"
                                        "Could not open calibration save file " << jsonCalFilepath;
                retVal = GC_ERR;
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::Save] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS CalibOctagon::CalcOctoWorldPoints( const double sideLength, std::vector< cv::Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        pts.clear();
        double cornerLength = sqrt( sideLength * sideLength / 2.0 );

#if 1
//        pts.push_back( Point2d( 0.0, 0.0 ) );
//        pts.push_back( Point2d( sideLength, 0.0 ) );
//        pts.push_back( Point2d( sideLength + cornerLength, cornerLength ) );
//        pts.push_back( Point2d( sideLength + cornerLength, sideLength + cornerLength ) );
//        pts.push_back( Point2d( sideLength, cornerLength + cornerLength + sideLength ) );
//        pts.push_back( Point2d( 0.0, cornerLength + cornerLength + sideLength ) );
//        pts.push_back( Point2d( -cornerLength, cornerLength + sideLength ) );
//        pts.push_back( Point2d( -cornerLength, cornerLength ) );
        pts.push_back( Point2d( 0.0, 0.0 ) );
        pts.push_back( Point2d( sideLength, 0.0 ) );
        pts.push_back( Point2d( sideLength + cornerLength, -cornerLength ) );
        pts.push_back( Point2d( sideLength + cornerLength, -sideLength - cornerLength ) );
        pts.push_back( Point2d( sideLength, -cornerLength - cornerLength - sideLength ) );
        pts.push_back( Point2d( 0.0, -cornerLength + -cornerLength - sideLength ) );
        pts.push_back( Point2d( -cornerLength, -cornerLength - sideLength ) );
        pts.push_back( Point2d( -cornerLength, -cornerLength ) );
#else
        pts.push_back( Point2d( 0.0, cornerLength + cornerLength + sideLength ) );
        pts.push_back( Point2d( sideLength, cornerLength + cornerLength + sideLength ) );
        pts.push_back( Point2d( sideLength + cornerLength, + cornerLength + sideLength ) );
        pts.push_back( Point2d( sideLength + cornerLength, + cornerLength ) );
        pts.push_back( Point2d( sideLength, 0.0 ) );
        pts.push_back( Point2d( 0.0, 0.0 ) );
        pts.push_back( Point2d( -cornerLength, cornerLength ) );
        pts.push_back( Point2d( -cornerLength, cornerLength + sideLength ) );
#endif
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcWorldPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::FindCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
//        vector< vector< Point > > conts;
//        conts.push_back( contour );
//        drawContours( mask, conts, -1, Scalar(128), 3);
//        imwrite( "/var/tmp/gaugecam/mask_conts.png", mask );

        if ( contour.size() < MIN_SYMBOL_CONTOUR_SIZE )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagonCorners] Contour must have at least " << MIN_SYMBOL_CONTOUR_SIZE << " contour points";
            retVal = GC_ERR;
        }
        else if ( mask.empty() || CV_8UC1 != mask.type() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagonCorners] Invalid mask image";
            retVal = GC_ERR;
        }
        else
        {
            Mat edges = Mat::zeros( mask.size(), CV_8UC1 );
            drawContours( edges, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), 1 );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            imwrite( DEBUG_FOLDER + "candidate_contour1.png", edges );
            imwrite( DEBUG_FOLDER + "candidate_contour_mask1.png", mask );
#endif
            Rect bb = boundingRect( contour );
            int swathSize = bb.height / 5;
            RotatedRect rotRect = fitEllipse( contour );
            Mat scratch = Mat::zeros( mask.size(), CV_8UC1 );
            line( scratch, Point( cvRound( rotRect.center.x ), cvRound( rotRect.center.y ) ), Point( 0, cvRound( rotRect.center.y ) ), Scalar( 255 ), swathSize );
            scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_FOLDER + "left_edge_pts_swath.png", scratch );
#endif

            int top = cvRound( rotRect.center.y ) - swathSize / 2;
            top = 0 > top ? 0 : top;
            int bot = cvRound( rotRect.center.y ) + swathSize / 2;
            bot = scratch.rows <= bot ? scratch.rows - 1 : bot;

            Rect rect( 0, top, cvRound( rotRect.center.x ), bot - top );
            Point2d lftPt1, lftPt2;
            retVal = GetLineEndPoints( scratch, rect, lftPt1, lftPt2 );
            if ( GC_OK == retVal )
            {
                scratch = 0;
                line( scratch, rotRect.center, Point( scratch.cols - 1, cvRound( rotRect.center.y ) ), Scalar( 255 ), swathSize );
                scratch &= edges;

                rect = Rect( cvRound( rotRect.center.x ), top, scratch.cols - cvRound( rotRect.center.x ), bot - top );
                Point2d rgtPt1, rgtPt2;
                retVal = GetLineEndPoints( scratch, rect, rgtPt1, rgtPt2 );
                if ( GC_OK == retVal )
                {
                    scratch = 0;
                    line( scratch, rotRect.center, Point( cvRound( rotRect.center.x ), 0 ), Scalar( 255 ), swathSize );
                    scratch &= edges;

                    int lft = cvRound( rotRect.center.x ) - swathSize / 2;
                    lft = 0 > lft ? 0 : lft;
                    int rgt = cvRound( rotRect.center.x ) + swathSize / 2;
                    rgt = scratch.cols <= rgt ? scratch.cols - 1 : rgt;

                    rect = Rect( lft, 0, rgt - lft, cvRound( rotRect.center.y ) );
                    Point2d topPt1, topPt2;
                    retVal = GetLineEndPoints( scratch, rect, topPt1, topPt2 );
                    if ( GC_OK == retVal )
                    {
                        scratch = 0;
                        line( scratch, rotRect.center, Point( cvRound( rotRect.center.x ), scratch.rows - 1 ), Scalar( 255 ), swathSize );
                        scratch &= edges;

                        rect = Rect( lft, cvRound( rotRect.center.y ), rgt - lft, scratch.rows - cvRound( rotRect.center.y ) );
                        Point2d botPt1, botPt2;
                        retVal = GetLineEndPoints( scratch, rect, botPt1, botPt2 );
                        if ( GC_OK == retVal )
                        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            line( color, lftPt1, lftPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, rgtPt1, rgtPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, topPt1, topPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, botPt1, botPt2, Scalar( 0, 0, 255 ), 1 );
#endif
                            retVal = LineIntersection( OctagonLine( topPt1, topPt2 ), OctagonLine( lftPt1, lftPt2 ), octoLines.top.pt1 );
                            if ( GC_OK == retVal )
                            {
                                octoLines.left.pt2 = octoLines.top.pt1;
                                retVal = LineIntersection( OctagonLine( topPt1, topPt2 ), OctagonLine( rgtPt1, rgtPt2 ), octoLines.top.pt2 );
                                if ( GC_OK == retVal )
                                {
                                    octoLines.right.pt1 = octoLines.top.pt2;
                                    retVal = LineIntersection( OctagonLine( botPt1, botPt2 ), OctagonLine( lftPt1, lftPt2 ), octoLines.bot.pt2 );
                                    if ( GC_OK == retVal )
                                    {
                                        octoLines.left.pt1 = octoLines.bot.pt2;
                                        retVal = LineIntersection( OctagonLine( botPt1, botPt2 ), OctagonLine( rgtPt1, rgtPt2 ), octoLines.right.pt2 );
                                        if ( GC_OK == retVal )
                                        {
                                            octoLines.bot.pt1 = octoLines.right.pt2;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_FOLDER + "symbol_edges.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        cout << e.what() << endl;
        FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagonCorners] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::CalcCorners( const OctagonLines octoLines, std::vector< cv::Point2d > &symbolCorners )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Point2d pt;
        symbolCorners.clear();
        retVal = LineIntersection( octoLines.topLeft, octoLines.top, pt );
        if ( GC_OK == retVal )
        {
            symbolCorners.push_back( pt );
            retVal = LineIntersection( octoLines.top, octoLines.topRight, pt );
            if ( GC_OK == retVal )
            {
                symbolCorners.push_back( pt );
                retVal = LineIntersection( octoLines.topRight, octoLines.right, pt );
                if ( GC_OK == retVal )
                {
                    symbolCorners.push_back( pt );
                    retVal = LineIntersection( octoLines.right, octoLines.botRight, pt );
                    if ( GC_OK == retVal )
                    {
                        symbolCorners.push_back( pt );
                        retVal = LineIntersection( octoLines.botRight, octoLines.bot, pt );
                        if ( GC_OK == retVal )
                        {
                            symbolCorners.push_back( pt );
                            retVal = LineIntersection( octoLines.bot, octoLines.botLeft, pt );
                            if ( GC_OK == retVal )
                            {
                                symbolCorners.push_back( pt );
                                retVal = LineIntersection( octoLines.botLeft, octoLines.left, pt );
                                if ( GC_OK == retVal )
                                {
                                    symbolCorners.push_back( pt );
                                    retVal = LineIntersection( octoLines.left, octoLines.topLeft, pt );
                                    if ( GC_OK == retVal )
                                    {
                                        symbolCorners.push_back( pt );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcCorners] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
GC_STATUS CalibOctagon::LineIntersection( const OctagonLine line1, const OctagonLine line2, Point2d &r )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d x = line2.pt1 - line1.pt1;
        Point2d d1 = line1.pt2 - line1.pt1;
        Point2d d2 = line2.pt2 - line2.pt1;

        double cross = d1.x * d2.y - d1.y * d2.x;
        if (abs(cross) < numeric_limits< double >::epsilon() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::LineIntersection] Lines are parallel";
            return GC_ERR;
        }

        double t1 = ( x.x * d2.y - x.y * d2.x ) / cross;
        r = line1.pt1 + d1 * t1;
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::LineIntersection] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::FindDiagonals( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( contour.size() < MIN_SYMBOL_CONTOUR_SIZE )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagonCorners] Contour must have at least " << MIN_SYMBOL_CONTOUR_SIZE << " contour points";
            retVal = GC_ERR;
        }
        else if ( mask.empty() || CV_8UC1 != mask.type() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalibOctagonCorners] Invalid mask image";
            retVal = GC_ERR;
        }
        else
        {
            Mat edges = Mat::zeros( mask.size(), CV_8UC1 );
            drawContours( edges, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), 1 );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            imwrite( DEBUG_FOLDER + "candidate_contour_mask.png", mask );
            imwrite( DEBUG_FOLDER + "candidate_contour.png", edges );
#endif
            Rect bb = boundingRect( contour );
            int swathSize = bb.height / 5;
            RotatedRect rotRect = fitEllipse( contour );
            Mat scratch = Mat::zeros( mask.size(), CV_8UC1 );
            line( scratch, rotRect.center, octoLines.top.pt1, Scalar( 255 ), swathSize );
            scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_FOLDER + "top_left_edge_pts_swath.png", scratch );
#endif

            Rect rect( cvRound( octoLines.top.pt1.x ), cvRound( octoLines.top.pt1.y ),
                       cvRound( rotRect.center.x - octoLines.top.pt1.x ),
                       cvRound( rotRect.center.y - octoLines.top.pt1.y ) );

            retVal = GetLineEndPoints( scratch, rect, octoLines.topLeft.pt1, octoLines.topLeft.pt2 );
            if ( GC_OK == retVal )
            {
                scratch = 0;
                line( scratch, rotRect.center, octoLines.top.pt2, Scalar( 255 ), swathSize );
                scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                imwrite( DEBUG_FOLDER + "top_right_edge_pts_swath.png", scratch );
#endif

                rect = Rect( cvRound( rotRect.center.x ), cvRound( octoLines.top.pt2.y ),
                             cvRound( octoLines.top.pt2.x - rotRect.center.x ),
                             cvRound( rotRect.center.y - octoLines.top.pt2.y ) );

                retVal = GetLineEndPoints( scratch, rect, octoLines.topRight.pt1, octoLines.topRight.pt2 );
                if ( GC_OK == retVal )
                {
                    scratch = 0;
                    line( scratch, rotRect.center, octoLines.bot.pt2, Scalar( 255 ), swathSize );
                    scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                    imwrite( DEBUG_FOLDER + "bot_left_edge_pts_swath.png", scratch );
#endif

                    rect = Rect( cvRound( octoLines.bot.pt2.x ), cvRound( rotRect.center.y ),
                                 cvRound( rotRect.center.x - octoLines.bot.pt2.x ),
                                 cvRound( octoLines.bot.pt2.y - rotRect.center.y ) );
                    retVal = GetLineEndPoints( scratch, rect, octoLines.botLeft.pt1, octoLines.botLeft.pt2 );
                    if ( GC_OK == retVal )
                    {
                        scratch = 0;
                        line( scratch, rotRect.center, octoLines.bot.pt1, Scalar( 255 ), swathSize );
                        scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                        imwrite( DEBUG_FOLDER + "bot_right_edge_pts_swath.png", scratch );
#endif

                        rect = Rect( cvRound( rotRect.center.x ), cvRound( rotRect.center.y ),
                                     cvRound( octoLines.bot.pt1.x - rotRect.center.x ),
                                     cvRound( octoLines.bot.pt1.y - rotRect.center.y ) );

                        retVal = GetLineEndPoints( scratch, rect, octoLines.botRight.pt1, octoLines.botRight.pt2 );
                        if ( GC_OK == retVal )
                        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            line( color, octoLines.topLeft.pt1, octoLines.topLeft.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.topRight.pt1, octoLines.topRight.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.botLeft.pt1, octoLines.botLeft.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.botRight.pt1, octoLines.botRight.pt2, Scalar( 0, 0, 255 ), 1 );
#endif
                        }
                    }
                }
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_FOLDER + "symbol_edges_diagonal.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::FindDiagonals] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::GetLineEndPoints( cv::Mat &mask, const cv::Rect rect, cv::Point2d &pt1, cv::Point2d &pt2 )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat search = mask( rect );
#ifdef DEBUG_FIND_CALIB_SYMBOL
        imwrite( DEBUG_FOLDER + "pt_search_img.png", mask );
        imwrite( DEBUG_FOLDER + "pt_search_rect.png", search );
#endif

        vector< Point > pts;
        retVal = GetNonZeroPoints( search, pts );
        if ( GC_OK == retVal )
        {
            for ( size_t i = 0; i < pts.size(); ++i )
            {
                pts[ i ].x += rect.x;
                pts[ i ].y += rect.y;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            drawContours( color, vector< vector< Point > >( 1, pts ), -1, Scalar( 0, 255, 255 ), 1 );
            imwrite( DEBUG_FOLDER + "pt_search_pts.png", color );
#endif

            Vec4d lne;
            fitLine( pts, lne, DIST_L12, 0.0, 0.01, 0.01 );

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // line equation: y = mx + b
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Point2d pt1x0, pt1y0, pt2x0, pt2y0;

            double a = lne[ 1 ];
            double b = -lne[ 0 ];
            double c = lne[ 0 ] * lne[ 3 ] - lne[ 1 ] * lne[ 2 ];

            double denom = ( 0.0 == a ? std::numeric_limits< double >::epsilon() : a );
            pt1y0.y = 0.0;
            pt1y0.x = c / -denom;
            pt2y0.y = static_cast< double >( mask.rows - 1 );
            pt2y0.x = ( b * pt2y0.y + c ) / -denom;

            denom = ( 0.0 == b ? std::numeric_limits< double >::epsilon() : b );
            pt1x0.x = 0.0;
            pt1x0.y = c / -denom;
            pt2x0.x = static_cast< double >( mask.cols - 1 );
            pt2x0.y = ( a * pt2x0.x + c ) / -denom;

            if ( 0.0 <= pt1y0.x && 0.0 <= pt1y0.y && mask.cols > pt1y0.x && mask.rows > pt1y0.y )
            {
                pt1 = pt1y0;
            }
            else
            {
                pt1 = pt1x0;
            }


            if ( 0.0 <= pt2y0.x && 0.0 <= pt2y0.y && mask.cols > pt2y0.x && mask.rows > pt2y0.y )
            {
                pt2 = pt2y0;
            }
            else
            {
                pt2 = pt2x0;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            line( color, pt1, pt2, Scalar( 0, 255, 0 ), 1 );
            imwrite( DEBUG_FOLDER + "pt_search_line.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::GetLineEndPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;

}
GC_STATUS CalibOctagon::GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::GetNonZeroPoints] Can not get points from an empty image";
            retVal = GC_ERR;
        }
        else
        {
            pts.clear();
            uchar *pPix;
            for( int r = 0; r < img.rows; ++r )
            {
                pPix = img.ptr< uchar >( r );
                for ( int c = 0; c < img.cols; ++c )
                {
                    if ( 0 != pPix[ c ] )
                        pts.push_back( Point( c, r ) );
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::GetNonZeroPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::PixelToWorld( const Point2d ptPixel, Point2d &ptWorld )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogPixToWorld.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::PixelToWorld] No calibration for pixel to world conversion";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptPixel );
            perspectiveTransform( vecIn, vecOut, matHomogPixToWorld );
            ptWorld = vecOut[ 0 ];
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::PixelToWorld] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::WorldToPixel( const Point2d ptWorld, Point2d &ptPixel )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::WorldToPixel]"
                                    "No calibration for world to pixel conversion";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptWorld );
            perspectiveTransform( vecIn, vecOut, matHomogWorldToPix );
            ptPixel = vecOut[ 0 ];
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::WorldToPixel] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::DrawOverlay( const cv::Mat &img, cv::Mat &result, const bool drawCalibScale,
                                     const bool drawCalibGrid, const bool drawSearchROI, const bool drawTargetSearchROI )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( matHomogPixToWorld.empty() || matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::DrawCalibration] System not calibrated";
            retVal = GC_ERR;
        }
        else if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::DrawCalibration] Empty image";
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
                FILE_LOG( logERROR ) << "[CalibOctagon::DrawCalibration] Invalid image type";
                retVal = GC_ERR;
            }

            int textStroke = std::max( 1, cvRound( static_cast< double >( result.rows ) / 300.0 ) );
            if ( GC_OK == retVal )
            {
                double dim = static_cast< double >( std::max( result.cols, result.rows ) );
                int lineWidth = max( 1, cvRound( dim / 900.0 ) );
                int targetRadius = lineWidth * 5;
                double fontScale = 1.0 + static_cast< double >( result.rows ) / 1200.0;
                if ( drawCalibScale || drawCalibGrid )
                {
                    if ( !model.validCalib  )
                    {
                        if ( -1 == model.targetSearchRegion.x )
                        {
                            line( result, Point( model.imgSize.width >> 2, model.imgSize.height >> 2 ),
                                  Point( 3 * ( model.imgSize.width >> 2 ), 3 * ( model.imgSize.height >> 2 ) ),
                                  Scalar( 0, 0, 255 ), textStroke );
                            line( result, Point( 3 * ( model.imgSize.width >> 2 ), model.imgSize.height >> 2 ),
                                  Point( model.imgSize.width >> 2, 3 * ( model.imgSize.height >> 2 ) ),
                                  Scalar( 0, 0, 255 ), textStroke );
                        }
                        else
                        {
                            line( result, Point( model.targetSearchRegion.x, model.targetSearchRegion.y ),
                                  Point( model.targetSearchRegion.x + model.targetSearchRegion.width,
                                         model.targetSearchRegion.y + model.targetSearchRegion.height ),
                                  Scalar( 0, 0, 255 ), textStroke );
                            line( result, Point( model.targetSearchRegion.x + model.targetSearchRegion.width, model.targetSearchRegion.y ),
                                  Point( model.targetSearchRegion.x, model.targetSearchRegion.y + model.targetSearchRegion.height ),
                                  Scalar( 0, 0, 255 ), textStroke );
                        }
                    }
                    else
                    {
                        line( result, Point2d( model.pixelPoints[ 0 ].x - targetRadius, model.pixelPoints[ 0 ].y ),
                                      Point2d( model.pixelPoints[ 0 ].x + targetRadius, model.pixelPoints[ 0 ].y ), Scalar( 0, 255, 0 ), lineWidth );
                        line( result, Point2d( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y - targetRadius ),
                                      Point2d( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y + targetRadius ), Scalar( 0, 255, 0 ), lineWidth );
                        circle( result, model.pixelPoints[ 0 ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                        for ( size_t i = 1; i < model.pixelPoints.size(); ++i )
                        {
                            line( result, model.pixelPoints[ i - 1 ], model.pixelPoints[ i ], Scalar( 255, 0, 0 ), lineWidth );
                            line( result, Point2d( model.pixelPoints[ i ].x - targetRadius, model.pixelPoints[ i ].y ),
                                          Point2d( model.pixelPoints[ i ].x + targetRadius, model.pixelPoints[ i ].y ), Scalar( 0, 255, 0 ), lineWidth );
                            line( result, Point2d( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y - targetRadius ),
                                          Point2d( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y + targetRadius ), Scalar( 0, 255, 0 ), lineWidth );
                            circle( result, model.pixelPoints[ i ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                        }
                        line( result, model.pixelPoints[ 0 ], model.pixelPoints[ model.pixelPoints.size() - 1 ], Scalar( 255, 0, 0 ), lineWidth );

                        if ( true )
                        {
                            line( result, Point2d( model.oldPixelPoints[ 0 ].x - targetRadius, model.oldPixelPoints[ 0 ].y ),
                                 Point2d( model.oldPixelPoints[ 0 ].x + targetRadius, model.oldPixelPoints[ 0 ].y ), Scalar( 255, 255, 0 ), lineWidth );
                            line( result, Point2d( model.oldPixelPoints[ 0 ].x, model.oldPixelPoints[ 0 ].y - targetRadius ),
                                 Point2d( model.oldPixelPoints[ 0 ].x, model.oldPixelPoints[ 0 ].y + targetRadius ), Scalar( 255, 255, 0 ), lineWidth );
                            circle( result, model.oldPixelPoints[ 0 ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                            for ( size_t i = 1; i < model.oldPixelPoints.size(); ++i )
                            {
                                line( result, model.oldPixelPoints[ i - 1 ], model.oldPixelPoints[ i ], Scalar( 255, 255, 0 ), lineWidth );
                                line( result, Point2d( model.oldPixelPoints[ i ].x - targetRadius, model.oldPixelPoints[ i ].y ),
                                     Point2d( model.oldPixelPoints[ i ].x + targetRadius, model.oldPixelPoints[ i ].y ), Scalar( 255, 255, 0 ), lineWidth );
                                line( result, Point2d( model.oldPixelPoints[ i ].x, model.oldPixelPoints[ i ].y - targetRadius ),
                                     Point2d( model.oldPixelPoints[ i ].x, model.oldPixelPoints[ i ].y + targetRadius ), Scalar( 255, 255, 0 ), lineWidth );
                                circle( result, model.oldPixelPoints[ i ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                            }
                            line( result, model.oldPixelPoints[ 0 ], model.oldPixelPoints[ model.oldPixelPoints.size() - 1 ], Scalar( 255, 255, 0 ), lineWidth );
                        }

                        if ( drawCalibScale )
                        {
                            double lftX = ( model.searchLineSet[ 0 ].top.x + model.searchLineSet[ 0 ].bot.x ) / 2.0;
                            double rgtX = ( model.searchLineSet[ model.searchLineSet.size() - 1 ].top.x +
                                    model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.x ) / 2.0;
                            double quarterLength = ( rgtX - lftX ) / 4.0;
                            lftX += quarterLength;
                            rgtX -= quarterLength;
                            quarterLength = ( rgtX - lftX ) / 4.0;

                            double centerPoint = ( lftX + rgtX ) / 2.0;
                            double startPoint = ( model.searchLineSet[ 0 ].top.y + model.searchLineSet[ model.searchLineSet.size() - 1 ].top.y ) / 2.0;
                            double endPoint = ( model.searchLineSet[ 0 ].bot.y + model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.y ) / 2.0;

                            char msg[ 256 ];
                            double yPos;
                            Point2d worldPt;
                            double vertInc = ( endPoint - startPoint ) / 10.0;
                            for ( int i = 0; i < 10; ++i )
                            {
                                yPos = startPoint + static_cast< double >( i ) * vertInc;
                                retVal = PixelToWorld( Point2d( centerPoint, yPos ), worldPt );
                                if ( GC_OK == retVal )
                                {
                                    sprintf( msg, "%.1f", worldPt.y );
                                    if ( 0 == i % 2 )
                                    {
                                        line( result, Point2d( lftX, yPos ), Point2d( rgtX, yPos ), Scalar( 0, 255, 255 ), lineWidth );
                                    }
                                    else
                                    {
                                        line( result, Point2d( lftX + quarterLength, yPos ),
                                              Point2d( rgtX - quarterLength, yPos ), Scalar( 0, 255, 255 ), lineWidth );

                                    }
                                    putText( result, msg, Point( cvRound( lftX - 120 ), cvRound( yPos ) + 15 ),
                                             FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 0, 255 ), lineWidth );
                                }
                            }
                        }
                        else
                        {
                            std::vector< OctagonLine > horzLines, vertLines;
                            retVal = CalcGridDrawPoints( horzLines, vertLines );
                            if ( GC_OK == retVal )
                            {
                                char msg[ 256 ];
                                Point2d ptWorld;

                                // zero level line
                                putText( result, "0.0", Point( 10, cvRound( horzLines[ 0 ].pt1.y - 10 ) ),
                                         FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 0, 255 ), lineWidth );
                                line( result, horzLines[ 0 ].pt1, horzLines[ 0 ].pt2, Scalar( 0, 0, 255 ), lineWidth );

                                // the rest of the horizontal lines
                                for ( size_t i = 1; i < horzLines.size(); ++i )
                                {
                                    retVal = PixelToWorld( horzLines[ i ].pt1, ptWorld );
                                    if ( GC_OK == retVal )
                                    {
                                        sprintf( msg, "%.1f", ptWorld.y );
                                        putText( result, msg, Point( 10, cvRound( horzLines[ i ].pt1.y - 10 ) ),
                                                 FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 255, 255 ), lineWidth );
                                        line( result, horzLines[ i ].pt1, horzLines[ i ].pt2, Scalar( 0, 255, 255 ), lineWidth );
                                    }
                                }

                                // vertical lines
                                for ( size_t i = 1; i < vertLines.size(); ++i )
                                {
                                    retVal = PixelToWorld( vertLines[ i ].pt1, ptWorld );
                                    if ( GC_OK == retVal )
                                    {
                                        line( result, vertLines[ i ].pt1, vertLines[ i ].pt2, Scalar( 0, 255, 255 ), lineWidth );
                                    }
                                }
                            }
                        }
                    }
                }
                if ( drawTargetSearchROI )
                {
                    rectangle( result, model.targetSearchRegion, Scalar( 255, 0, 0 ), textStroke );
                }
                if ( drawSearchROI )
                {
                    if ( model.searchLineSet.empty() )
                    {
                        putText( result, "NO SEARCH REGION SET", Point( 50, result.cols - 100 ), FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 0, 255 ), 3 );
                        rectangle( result, Rect( 100, 100, result.cols - 200, result.rows - 200 ), Scalar( 0, 0, 255 ), 3 );
                        line( result, Point( 100, 100 ), Point( result.cols - 200, result.rows - 200 ), Scalar( 0, 0, 255 ), 3 );
                        line( result, Point( 100, result.rows - 200 ), Point( result.cols - 200, 100 ), Scalar( 0, 0, 255 ), 3 );
                    }
                    else
                    {
                        line( result, model.searchLineSet[ 0 ].top, model.searchLineSet[ 0 ].bot, Scalar( 255, 0, 0 ), textStroke );
                        line( result, model.searchLineSet[ 0 ].top, model.searchLineSet[ model.searchLineSet.size() - 1 ].top, Scalar( 255, 0, 0 ), textStroke );
                        line( result, model.searchLineSet[ model.searchLineSet.size() - 1 ].top, model.searchLineSet[ model.searchLineSet.size() - 1 ].bot, Scalar( 255, 0, 0 ), textStroke );
                        line( result, model.searchLineSet[ 0 ].bot, model.searchLineSet[ model.searchLineSet.size() - 1 ].bot, Scalar( 255, 0, 0 ), textStroke );

                        if ( 4 == model.waterlineSearchCornersAdj.size() )
                        {
                            polylines( result, model.waterlineSearchCornersAdj, true, Scalar( 0, 0, 255 ), std::max( textStroke >> 1, 1 ) );
                        }
                    }
                }
            }
            else
            {
                putText( result, "CALIBRATION NOT VALID", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), textStroke );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::DrawCalibration] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::GetXEdgeMinDiffX(const double xWorld, cv::Point2d &ptPix, const bool isTopSideY )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double x_min = 0;
        double diff, minDiff;

        Point2d ptWorld;
        double y_pos = isTopSideY ? model.imgSize.height - 1 : 0;
        retVal = PixelToWorld( Point2d( y_pos , 0.0 ), ptWorld );
        if ( GC_OK == retVal )
        {
            minDiff = fabs( xWorld - ptWorld.y );
            for ( int i = 1; i < model.imgSize.width; ++i )
            {
                retVal = PixelToWorld( Point2d( i, y_pos ), ptWorld );
                if ( GC_OK == retVal )
                {
                    diff = fabs( xWorld - ptWorld.x );
                    if ( minDiff > diff )
                    {
                        x_min = i;
                        minDiff = diff;
                    }
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                ptPix = Point2d( x_min, y_pos );
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::GetMinWorldDiffY] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::GetXEdgeMinDiffY(const double yWorld, cv::Point2d &ptPix, const bool isRightSideX )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double y_min = 0;
        double diff, minDiff;

        Point2d ptWorld;
        double x_pos = isRightSideX ? model.imgSize.width - 1 : 0;
        retVal = PixelToWorld( Point2d( x_pos, 0.0 ), ptWorld );
        if ( GC_OK == retVal )
        {
            minDiff = fabs( yWorld - ptWorld.y );
            for ( int i = 1; i < model.imgSize.height; ++i )
            {
                retVal = PixelToWorld( Point2d( x_pos, i ), ptWorld );
                if ( GC_OK == retVal )
                {
                    diff = fabs( yWorld - ptWorld.y );
                    if ( minDiff > diff )
                    {
                        y_min = i;
                        minDiff = diff;
                    }
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                ptPix = Point2d( x_pos, y_min );
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::GetMinWorldDiffY] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::CalcGridDrawPoints( std::vector< OctagonLine > &horzLines, std::vector< OctagonLine > &vertLines )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 8 != model.pixelPoints.size() || 8 != model.worldPoints.size() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalcGridDrawPoints] System not calibrated";
            retVal = GC_ERR;
        }
        else
        {
            horzLines.clear();
            vertLines.clear();

            double horzInc = static_cast< double >( model.imgSize.width ) / 11.0;
            double vertInc = static_cast< double >( model.imgSize.height ) / 11.0;

            Point2d ptLft, ptRgt;
            double slope = ( model.pixelPoints[ 5 ].y - model.pixelPoints[ 4 ].y ) /
                           ( model.pixelPoints[ 5 ].x - model.pixelPoints[ 4 ].x );
            ptLft.y = model.pixelPoints[ 4 ].y + slope * model.pixelPoints[ 4 ].x;
            ptLft.x = 0.0;

            ptRgt.x = model.imgSize.width - 1;
            ptRgt.y = slope * model.pixelPoints[ 4 ].x + ptLft.y;

            Point2d ptPix1, ptPix2;
            retVal = GetXEdgeMinDiffY( 0.0, ptPix1, false );
            if ( GC_OK == retVal )
            {
                retVal = GetXEdgeMinDiffY( 0.0, ptPix2, true );
                if ( GC_OK == retVal )
                {
                    horzLines.push_back( OctagonLine( ptPix1, ptPix2 ) );
                }
            }

            double pixLftZeroY = ptPix1.y;

            Point2d ptWorld;
            if ( GC_OK == retVal )
            {
                double y_pixPos = pixLftZeroY - vertInc;
                while( 0.0 <= y_pixPos )
                {
                    retVal = PixelToWorld( Point2d( 0.0, y_pixPos ), ptWorld );
                    if ( GC_OK == retVal )
                    {
                        retVal = GetXEdgeMinDiffY( ptWorld.y, ptPix1, false );
                        if ( GC_OK == retVal )
                        {
                            retVal = GetXEdgeMinDiffY( ptWorld.y, ptPix2, true );
                            if ( GC_OK == retVal )
                            {
                                horzLines.push_back( OctagonLine( ptPix1, ptPix2 ) );
                            }
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        break;
                    }
                    y_pixPos -= vertInc;
                }
            }

            if ( GC_OK == retVal )
            {
                double y_pixPos = pixLftZeroY + vertInc;
                while( model.imgSize.height > y_pixPos )
                {
                    retVal = PixelToWorld( Point2d( 0.0, y_pixPos ), ptWorld );
                    if ( GC_OK == retVal )
                    {
                        retVal = GetXEdgeMinDiffY( ptWorld.y, ptPix1, false );
                        if ( GC_OK == retVal )
                        {
                            retVal = GetXEdgeMinDiffY( ptWorld.y, ptPix2, true );
                            if ( GC_OK == retVal )
                            {
                                horzLines.push_back( OctagonLine( ptPix1, ptPix2 ) );
                            }
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        break;
                    }
                    y_pixPos += vertInc;
                }
            }

            if ( GC_OK == retVal )
            {
                double x_pixPos = horzInc;
                while( model.imgSize.width > x_pixPos )
                {
                    retVal = PixelToWorld( Point2d( x_pixPos, 0.0 ), ptWorld );
                    if ( GC_OK == retVal )
                    {
                        retVal = GetXEdgeMinDiffX( ptWorld.x, ptPix1, false );
                        if ( GC_OK == retVal )
                        {
                            retVal = GetXEdgeMinDiffX( ptWorld.x, ptPix2, true );
                            if ( GC_OK == retVal )
                            {
                                vertLines.push_back( OctagonLine( ptPix1, ptPix2 ) );
                            }
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        break;
                    }
                    x_pixPos += horzInc;
                }
            }
        }

        if ( horzLines.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalcGridDrawPoints] Unable to calculate any grid lines";
            retVal = GC_ERR;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcGridDrawPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::CalcSearchROI( const double botLftPtToLft, const double botLftPtToTop,
                                        const double botLftPtToRgt, const double botLftPtToBot, cv::Point2d &lftTop,
                                        cv::Point2d &rgtTop, cv::Point2d &lftBot, cv::Point2d &rgtBot )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( model.worldPoints.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalcSearchROI] System not calibrated";
            retVal = GC_ERR;
        }
        else if ( ( model.worldPoints[ 1 ].y == model.worldPoints[ 4 ].y ) ||
                  ( model.worldPoints[ 0 ].y == model.worldPoints[ 5 ].y ) )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::CalcSearchROI] Invalid calibration";
            retVal = GC_ERR;
        }
        else
        {
#if 1
            Point2d botLftPt =  model.worldPoints[ 5 ];
            lftTop = botLftPt + Point2d( botLftPtToLft, botLftPtToTop );
            rgtTop = botLftPt + Point2d( botLftPtToRgt, botLftPtToTop );
            lftBot = botLftPt + Point2d( botLftPtToLft, botLftPtToBot );
            rgtBot = botLftPt + Point2d( botLftPtToRgt, botLftPtToBot );
#else
            lftBot = Point2d( ( model.worldPoints[ 5 ].x + model.worldPoints[ 6 ].x ) / 2.0, 0.0 );
            rgtBot = Point2d( ( model.worldPoints[ 3 ].x + model.worldPoints[ 4 ].x ) / 2.0, 0.0 );
            lftTop = Point2d( ( model.worldPoints[ 5 ].x + model.worldPoints[ 6 ].x ) / 2.0, botLftPtToZero * 0.9 );
            rgtTop = Point2d( ( model.worldPoints[ 3 ].x + model.worldPoints[ 4 ].x ) / 2.0, botLftPtToZero * 0.9 );
#endif
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::CalcSearchROI] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibOctagon::TestCalibration( bool &isValid )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        isValid = false;
        if ( !model.pixelPoints.empty() && !model.worldPoints.empty() &&
             model.pixelPoints.size() == model.worldPoints.size() )
        {
            double dist, distAvg = 0.0;
            double distMin = 9999999;
            double distMax = -9999999;
            for ( size_t i = 1; i < model.pixelPoints.size(); ++i )
            {
                dist = distance( model.pixelPoints[ i - 1 ], model.pixelPoints[ i ] );
                if ( dist < distMin )
                {
                    distMin = dist;
                }
                if ( dist > distMax )
                {
                    distMax = dist;
                }
                distAvg += dist;
            }
            dist = distance( model.pixelPoints[ 0 ], model.pixelPoints[ model.pixelPoints.size() - 1 ] );
            if ( dist < distMin )
            {
                distMin = dist;
            }
            if ( dist > distMax )
            {
                distMax = dist;
            }
            distAvg += dist;
            distAvg /= static_cast< double >( model.pixelPoints.size() );

            if ( distMax - distMin < 0.35 * distAvg )
            {
                isValid = true;
            }
            else
            {
                FILE_LOG( logERROR ) << "[CalibOctagon::TestCalibration] Calibration point find test bad";
                retVal = GC_ERR;
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibOctagon::TestCalibration] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

// helper functions
static double distance( Point2d a, Point2d b )
{
    return sqrt( ( b.y - a.y ) * ( b.y - a.y ) + ( b.x - a.x ) * ( b.x - a.x ) );
}
#if 0
static double elongation( Moments m )
{
    double x = m.mu20 + m.mu02;
    double y = 4 * m.mu11 * m.mu11 + ( m.mu20 - m.mu02 ) * ( m.mu20 - m.mu02 );
    double srt = sqrt( y );
    if ( x - srt > DBL_EPSILON )
        return ( x + srt ) / ( x - srt );
    else
        return 1.0;
}
#endif

} // namespace gc
