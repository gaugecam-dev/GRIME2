#include "log.h"
#include "calibexecutive.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <filesystem>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = std::filesystem;
namespace pt = property_tree;

static double Distance( const Point2d p1, const Point2d p2 )
{
    return sqrt( ( p2.x - p1.x ) * ( p2.x - p1.x ) +
                 ( p2.y - p1.y ) * ( p2.y - p1.y ) );
}

namespace gc
{

ostream &operator<<( ostream &out, CalibExecParams &params )
{

    out << "{ \"calibType\": \"" << params.calibType << "\", ";
    out << "\"calibWorldPt_csv\": \"" << params.worldPtCSVFilepath << "\", ";
    out << "\"facetLength\": " << params.facetLength << ", ";
    out << "\"zeroOffset\":" << params.zeroOffset << "," << endl;
    out << "\"botLftPtToLft\":" << params.botLftPtToLft << "," << endl;
    out << "\"botLftPtToTop\":" << params.botLftPtToTop << "," << endl;
    out << "\"botLftPtToRgt\":" << params.botLftPtToRgt << "," << endl;
    out << "\"botLftPtToBot\":" << params.botLftPtToBot << "," << endl;
    out << "\"calibResult_json\": \"" << params.calibResultJsonFilepath << "\", ";
    out << "\"drawCalibScale\": " << ( params.drawCalibScale ? 1 : 0 ) << ", ";
    out << "\"drawCalibGrid\": " << ( params.drawCalibGrid ? 1 : 0 ) << ", ";
    out << "\"drawMoveSearchROIs\": " << ( params.drawMoveSearchROIs ? 1 : 0 ) << ", ";
    out << "\"drawWaterLineSearchROI\": " << ( params.drawWaterLineSearchROI ? 1 : 0 ) << ", ";
    out << "\"drawTargetSearchROI\": " << ( params.drawTargetSearchROI ? 1 : 0 ) << ", ";
    out << "\"targetRoi_x\": " << ( params.targetSearchROI.x ) << ", ";
    out << "\"targetRoi_y\": " << ( params.targetSearchROI.y ) << ", ";
    out << "\"targetRoi_width\": " << ( params.targetSearchROI.width ) << ", ";
    out << "\"targetRoi_height\": " << ( params.targetSearchROI.height ) << " }";
    return out;
}

CalibExecutive::CalibExecutive()
{
}
void CalibExecutive::clear()
{
    bowTie.clear();
    stopSign.clear();
}
GC_STATUS CalibExecutive::GetTargetSearchROI( cv::Rect &rect )
{
    rect = "BowTie" == GetCalibType() ? bowTie.TargetRoi() : stopSign.TargetRoi();
    return GC_OK;
}
GC_STATUS CalibExecutive::GetCalibParams( std::string &calibParams )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == GetCalibType() )
    {
        retVal = bowTie.GetCalibParams( calibParams );
    }
    if ( "StopSign" == GetCalibType() )
    {
        retVal = stopSign.GetCalibParams( calibParams );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Recalibrate] No calibration defined" ;
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::Recalibrate( const Mat &img, const std::string calibType,
                                       double &rmseDist, double &rmseX, double &rmseY, string &err_msg )
{
    GC_STATUS retVal = GC_OK;

    string controlJson;
    if ( "StopSign" == calibType )
    {
        controlJson = stopSign.ControlJson();
    }
    else if ( "BowTie" == calibType )
    {
        controlJson = bowTie.ControlJson();
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Recalibrate] Invalid calibration type" ;
        retVal = GC_ERR;
    }

    if ( GC_OK == retVal )
    {
        retVal = Calibrate( img, controlJson, rmseDist, rmseX, rmseY, err_msg );
    }

    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                                     double &rmseDist, double &rmseX, double &rmseY, string &err_msg,
                                     const bool drawAll )
{
    GC_STATUS retVal = Calibrate( img, jsonParams, rmseDist, rmseX, rmseY, err_msg );
    if ( GC_OK == retVal )
    {
        retVal = DrawOverlay( img, imgResult, drawAll );
    }
    return retVal;
}
GC_STATUS CalibExecutive::SetCalibFromJson( const std::string &jsonParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        // cout << jsonParams << endl;
        paramsCurrent.clear();

        stringstream ss;
        ss << jsonParams;
        // cout << ss.str() << endl;

        pt::ptree top_level;
        pt::json_parser::read_json( ss, top_level );

        paramsCurrent.calibType = top_level.get< string >( "calibType", "" );
        paramsCurrent.worldPtCSVFilepath = top_level.get< string >( "calibWorldPt_csv", "" );
        paramsCurrent.facetLength = top_level.get< double >( "facetLength", -1.0 );

        paramsCurrent.botLftPtToLft = top_level.get< double >( "botLftPtToLft", -0.5 );
        paramsCurrent.botLftPtToTop = top_level.get< double >( "botLftPtToTop", 1.0 );
        paramsCurrent.botLftPtToRgt = top_level.get< double >( "botLftPtToRgt", 1.5 );
        paramsCurrent.botLftPtToBot = top_level.get< double >( "botLftPtToBot", -3.0 );

        paramsCurrent.moveSearchROIGrowPercent = cvRound( top_level.get< double >( "moveSearchROIGrowPercent", 0 ) );
        paramsCurrent.calibResultJsonFilepath = top_level.get< string >( "calibResult_json", "" );
        paramsCurrent.drawCalibScale = 1 == top_level.get< int >( "drawCalibScale", 0 );
        paramsCurrent.drawCalibGrid = 1 == top_level.get< int >( "drawCalibGrid", 0 );
        paramsCurrent.drawMoveSearchROIs = 1 == top_level.get< int >( "drawMoveSearchROIs", 0 );
        paramsCurrent.drawWaterLineSearchROI = 1 == top_level.get< int >( "drawWaterLineSearchROI", 0 );
        paramsCurrent.drawTargetSearchROI = 1 == cvRound( top_level.get< double >( "drawTargetSearchROI", 0 ) );
        paramsCurrent.targetSearchROI.x = top_level.get< int >( "targetRoi_x", -1 );
        paramsCurrent.targetSearchROI.y = top_level.get< int >( "targetRoi_y", -1 );
        paramsCurrent.targetSearchROI.width = top_level.get< int >( "targetRoi_width", -1 );
        paramsCurrent.targetSearchROI.height = top_level.get< int >( "targetRoi_height", -1 );

        paramsCurrent.lineSearch_lftTop.x = top_level.get< int >( "searchPoly_lftTop_x", -1 );
        paramsCurrent.lineSearch_lftTop.y = top_level.get< int >( "searchPoly_lftTop_y", -1 );
        paramsCurrent.lineSearch_rgtTop.x = top_level.get< int >( "searchPoly_rgtTop_x", -1 );
        paramsCurrent.lineSearch_rgtTop.y = top_level.get< int >( "searchPoly_rgtTop_y", -1 );
        paramsCurrent.lineSearch_lftBot.x = top_level.get< int >( "searchPoly_lftBot_x", -1 );
        paramsCurrent.lineSearch_lftBot.y = top_level.get< int >( "searchPoly_lftBot_y", -1 );
        paramsCurrent.lineSearch_rgtBot.x = top_level.get< int >( "searchPoly_rgtBot_x", -1 );
        paramsCurrent.lineSearch_rgtBot.y = top_level.get< int >( "searchPoly_rgtBot_y", -1 );

        if ( "StopSign" == paramsCurrent.calibType )
        {
            stopSign.Model().controlJson = jsonParams;
            stopSign.Model().facetLength = paramsCurrent.facetLength;
            // stopSign.Model().zeroOffset = paramsCurrent.zeroOffset;
            stopSign.Model().zeroOffset = top_level.get< double >( "zeroOffset", 0.0 );
            stopSign.Model().targetSearchRegion = paramsCurrent.targetSearchROI;
            stopSign.Model().waterlineSearchCorners.clear();

            stopSign.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_lftTop );
            stopSign.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_rgtTop );
            stopSign.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_lftBot );
            stopSign.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_rgtBot );
            sort( stopSign.Model().waterlineSearchCorners.begin(), stopSign.Model().waterlineSearchCorners.end(),
                  []( Point const &a, Point const &b ) { return ( a.y < b.y ); } );
            if ( stopSign.Model().waterlineSearchCorners[ 0 ].x > stopSign.Model().waterlineSearchCorners[ 1 ].x )
            {
                Point ptTemp = stopSign.Model().waterlineSearchCorners[ 0 ];
                stopSign.Model().waterlineSearchCorners[ 0 ] = stopSign.Model().waterlineSearchCorners[ 1 ];
                stopSign.Model().waterlineSearchCorners[ 1 ] = ptTemp;
            }
            if ( stopSign.Model().waterlineSearchCorners[ 2 ].x > stopSign.Model().waterlineSearchCorners[ 3 ].x )
            {
                Point ptTemp = stopSign.Model().waterlineSearchCorners[ 2 ];
                stopSign.Model().waterlineSearchCorners[ 2 ] = stopSign.Model().waterlineSearchCorners[ 3 ];
                stopSign.Model().waterlineSearchCorners[ 3 ] = ptTemp;
            }
        }
        else if ( "BowTie" == paramsCurrent.calibType )
        {
            bowTie.Model().controlJson = jsonParams;
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Invalid calibration type=" <<
                                    ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
            retVal = GC_ERR;
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::TestROIPositions( const int cols, const int rows )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 0 > paramsCurrent.lineSearch_lftTop.x || 0 > paramsCurrent.lineSearch_lftTop.y ||
             cols <= paramsCurrent.lineSearch_lftTop.x || rows <= paramsCurrent.lineSearch_lftTop.y ||
             0 > paramsCurrent.lineSearch_lftBot.x || 0 > paramsCurrent.lineSearch_lftBot.y ||
             cols <= paramsCurrent.lineSearch_lftBot.x || rows <= paramsCurrent.lineSearch_lftBot.y ||
             0 > paramsCurrent.lineSearch_rgtTop.x || 0 > paramsCurrent.lineSearch_rgtTop.y ||
             cols <= paramsCurrent.lineSearch_rgtTop.x || rows <= paramsCurrent.lineSearch_rgtTop.y ||
             0 > paramsCurrent.lineSearch_rgtBot.x || 0 > paramsCurrent.lineSearch_rgtBot.y ||
             cols <= paramsCurrent.lineSearch_rgtBot.x || rows <= paramsCurrent.lineSearch_rgtBot.y )
        {
            FILE_LOG( logERROR ) <<  "[CalibExecutive::TestROIPositions] Target or waterline search region out-of-bounds";
            retVal = GC_ERR;
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::TestROIPositions] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const cv::Mat &img, const std::string jsonParams,
                                     double &rmseDist, double &rmseX, double &rmseY, string &err_msg )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        string jsonParamsWhich = jsonParams;
        if ( jsonParamsWhich.empty() )
        {
            if ( "StopSign" == paramsCurrent.calibType )
            {
                if ( !stopSign.Model().controlJson.empty() )
                {
                    jsonParamsWhich = stopSign.Model().controlJson;
                }
                else
                {
                    err_msg = "CALIB FAIL: No available octagon calibration control string";
                    FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] No available octagon calibration control string";
                    retVal = GC_ERR;
                }
            }
            else if ( "BowTie" == paramsCurrent.calibType )
            {
                if ( !bowTie.Model().controlJson.empty() )
                {
                    jsonParamsWhich = bowTie.Model().controlJson;
                }
                else
                {
                    err_msg = "CALIB FAIL: No available bow tie calibration control string";
                    FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] No available bow tie calibration control string";
                    retVal = GC_ERR;
                }
            }
            else
            {
                err_msg = "CALIB FAIL: No available calibration control string";
                FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] No available calibration control string";
                retVal = GC_ERR;
            }
        }
        else
        {
            retVal = SetCalibFromJson( jsonParamsWhich );
        }

        if ( GC_OK != retVal )
        {
            err_msg = "CALIB FAIL: Could not set calib parameters from json string";
        }
        else
        {
            retVal = TestROIPositions( img.cols, img.rows );
            if ( GC_OK != retVal )
            {
                err_msg = "CALIB FAIL: Target or waterline search region out-of-bounds";
            }
            else
            {
                Mat imgFixed;
                Rect searchBB;
                if ( CV_8UC3 == img.type() )
                {
                    cvtColor( img, imgFixed, cv::COLOR_BGR2GRAY );
                }
                else
                {
                    imgFixed = img;
                }
                if ( "BowTie" == paramsCurrent.calibType )
                {
                    retVal = CalibrateBowTie( imgFixed, jsonParamsWhich, err_msg );
                    if ( GC_OK == retVal )
                    {
                        retVal = bowTie.GetSearchRegionBoundingRect( searchBB );
                    }
                }
                else if ( "StopSign" == paramsCurrent.calibType )
                {
                    retVal = CalibrateStopSign( imgFixed, jsonParamsWhich, err_msg );
                    if ( GC_OK == retVal )
                    {
                        retVal = stopSign.GetSearchRegionBoundingRect( searchBB );
                        if ( GC_OK != retVal )
                        {
                            err_msg = "CALIB FAIL: Octagon calibration search bounding box could not be set";
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        err_msg = "CALIB_FAIL: Octagon calibration failed";
                        retVal = GC_ERR;
                    }
                }
                else
                {

                    err_msg = "CALIB FAIL: Invalid calibration type=" + ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
                    FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Invalid calibration type=" <<
                                            ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
                    retVal = GC_ERR;
                }

                if ( GC_OK == retVal )
                {
                    retVal = CalculateRMSE( imgFixed( searchBB ), rmseDist, rmseX, rmseY );
                    if ( GC_OK != retVal )
                    {
                        rmseDist = rmseX = rmseY = -9999999.0;
                        FILE_LOG( logWARNING ) << "[CalibBowtie::Calibrate] Could not calculate RMSE";
                        retVal = GC_OK;
                    }
                }
            }
        }
    }
    catch( boost::exception &e )
    {
        err_msg = "CALIB FAIL: Exception";
        FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::AdjustStopSignForRotation( const Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle )
{
    GC_STATUS retVal = stopSign.AdjustStopSignForRotation( imgSize, calcLinePts, offsetAngle );
    return retVal;
}
GC_STATUS CalibExecutive::DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg )
{
    {
        GC_STATUS retVal = GC_OK;
        if ( "BowTie" == paramsCurrent.calibType )
        {
            // CalibModelBowtie model = bowTie.GetModel();
            retVal = bowTie.DrawAssocPts( img, overlay, err_msg );
        }
        else if ( "StopSign" == paramsCurrent.calibType )
        {
            retVal = stopSign.DrawAssocPts( img, overlay, err_msg );
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::DrawAssocPts] Invalid calibration type=" <<
                                    ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
            retVal = GC_ERR;
        }
        return retVal;
    }
}
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawAll )
{
    GC_STATUS retVal = GC_OK;
    if ( drawAll )
    {
        retVal = DrawOverlay( matIn, imgMatOut, false, true, true, true, true );
    }
    else
    {
        retVal = DrawOverlay( matIn, imgMatOut, paramsCurrent.drawCalibScale, paramsCurrent.drawCalibGrid,
                                        paramsCurrent.drawMoveSearchROIs, paramsCurrent.drawWaterLineSearchROI,
                                        paramsCurrent.drawTargetSearchROI );
    }
    return retVal;
}
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale, const bool drawCalibGrid,
                                       const bool drawMoveROIs, const bool drawSearchROI, const bool drawTargetROI, const Point2d moveOffset )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        // CalibModelBowtie model = bowTie.GetModel();
        retVal = bowTie.DrawOverlay( matIn, imgMatOut, drawCalibScale, drawCalibGrid, drawMoveROIs, drawSearchROI, drawTargetROI, moveOffset );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.DrawOverlay( matIn, imgMatOut, drawCalibScale, drawCalibGrid, drawMoveROIs, drawSearchROI, drawTargetROI );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::DrawOverlay] Invalid calibration type=" <<
                                ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = bowTie.PixelToWorld( pixelPt, worldPt );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.PixelToWorld( pixelPt, worldPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::PixelToWorld] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = bowTie.WorldToPixel( worldPt, pixelPt );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.WorldToPixel( worldPt, pixelPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::WorldToPixel] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibStopSign::GetSearchRegionBoundingRect( cv::Rect &rect )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( model.searchLineSet.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::GetSearchRegionBoundingRect] System not calibrated";
            retVal = GC_ERR;
        }
        else
        {
            int left = std::min( model.searchLineSet[ 0 ].top.x, model.searchLineSet[ 0 ].bot.x );
            int top = std::min( model.searchLineSet[ 0 ].top.y, model.searchLineSet[ model.searchLineSet.size() - 1 ].top.y );
            int right = std::max( model.searchLineSet[ model.searchLineSet.size() - 1 ].top.x, model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.x );
            int bottom = std::max( model.searchLineSet[ 0 ].bot.y, model.searchLineSet[ model.searchLineSet.size() - 1 ].bot.y );
            rect = Rect( left, top, right - left, bottom - top );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::GetSearchRegionBoundingRect] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< Point2d > > &worldCoords )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        ifstream file( csvFilepath );
        if ( !file.is_open() )
        {
            FILE_LOG( logERROR ) << "Could not open CSV filepath=" << csvFilepath;
            retVal = GC_ERR;
        }
        else
        {
            worldCoords.clear();

            string line;
            vector< string > vec;
            vector< Point2d > rowPts;

            getline( file, line );
            while ( getline( file, line ) )
            {
                rowPts.clear();
                algorithm::split( vec, line, is_any_of( "," ) );
                for ( size_t i = 0; i < vec.size(); i += 2 )
                {
                    rowPts.push_back( Point2d( atof( vec[ i ].c_str() ), atof( vec[ i + 1 ].c_str() ) ) );
                }
                worldCoords.push_back( rowPts );
            }
            file.close();
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::ReadWorldCoordsFromCSVBowTie] " << diagnostic_information( e );
        FILE_LOG( logERROR ) << "Could not read CSV filepath=" << csvFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateStopSign( const cv::Mat &img, const string &controlJson, string &err_msg )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = stopSign.Calibrate( img, controlJson, err_msg );
        if ( GC_OK != retVal )
        {
            Mat matIn;

            // GaussianBlur( matIn, matIn, Size( 5, 5 ), 1.0 );
            medianBlur( img, matIn, 7 );

            Mat kern = getStructuringElement( MORPH_ELLIPSE, Size( 5, 5 ) );
            erode( matIn, matIn, kern, Point( -1, -1 ), 1 );
            retVal = stopSign.Calibrate( matIn, controlJson, err_msg );
        }
        if ( GC_OK == retVal )
        {
            retVal = stopSign.Save( paramsCurrent.calibResultJsonFilepath );
        }
    }
    catch( Exception &e )
    {
        err_msg = "CALIB FAIL: Exception";
        FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateBowTie( const cv::Mat &img, const std::string &controlJson, string &err_msg )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, img.size() );
        if ( GC_OK != retVal )
        {
            err_msg = "CALIB FAIL: Could not initialize bowtie templates for calibration";
            FILE_LOG( logERROR ) << "[VisApp::VisApp] Could not initialize bowtie templates for calibration";
        }
        else
        {
            vector< vector< Point2d > > worldCoords;
            retVal = ReadWorldCoordsFromCSVBowTie( paramsCurrent.worldPtCSVFilepath, worldCoords );
            if ( GC_OK == retVal )
            {
#ifdef DEBUG_BOWTIE_FIND
                retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE, DEBUG_FOLDER + "bowtie_find.png" );
#else
                Rect rect = ( -1 == paramsCurrent.targetSearchROI.x ||
                              -1 == paramsCurrent.targetSearchROI.y ||
                              -1 == paramsCurrent.targetSearchROI.width ||
                              -1 == paramsCurrent.targetSearchROI.height ) ? Rect( 0, 0, img.cols, img.rows ) : paramsCurrent.targetSearchROI;
                retVal = findCalibGrid.FindTargets( img, rect, MIN_BOWTIE_FIND_SCORE );
#endif
                if ( GC_OK == retVal )
                {
                    vector< vector< Point2d > > pixelCoords;
                    retVal = findCalibGrid.GetFoundPoints( pixelCoords );
                    if ( GC_OK != retVal )
                    {
                        err_msg = "CALIB FAIL: Could get bowtie grid points";
                    }
                    else
                    {
                        if ( pixelCoords.size() != worldCoords.size() )
                        {
                            err_msg = "CALIB FAIL: Found pixel array row count does not equal world array count";
                            FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] Found pixel array row count does not equal world array count";
                            retVal = GC_ERR;
                        }
                        else
                        {
                            vector< Point2d > pixPtArray;
                            vector< Point2d > worldPtArray;
                            for ( size_t i = 0; i < pixelCoords.size(); ++i )
                            {
                                if ( pixelCoords[ i ].size() != worldCoords[ i ].size() )
                                {
                                    err_msg = "CALIB FAIL: Found pixel array column count does not equal world array count";
                                    FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] Found pixel array column count does not equal world array count";
                                    retVal = GC_ERR;
                                    break;
                                }
                                for ( size_t j = 0; j < pixelCoords[ i ].size(); ++j )
                                {
                                    pixPtArray.push_back( pixelCoords[ i ][ j ] );
                                    worldPtArray.push_back( worldCoords[ i ][ j ] );
                                }
                            }
                            vector< Point > searchLineCorners = { paramsCurrent.lineSearch_lftTop, paramsCurrent.lineSearch_rgtTop,
                                                                  paramsCurrent.lineSearch_lftBot, paramsCurrent.lineSearch_rgtBot };

                            retVal = bowTie.Calibrate( pixPtArray, worldPtArray,
                                                       static_cast< double >( paramsCurrent.moveSearchROIGrowPercent ) / 100.0,
                                                       controlJson, Size( 2, 4 ), img.size(), searchLineCorners, err_msg );
                            if ( GC_OK == retVal )
                            {
                                retVal = bowTie.Save( paramsCurrent.calibResultJsonFilepath );
                                if ( GC_OK != retVal )
                                {
                                    err_msg = "CALIB WARN: Could not save bowtie calib config to " + paramsCurrent.calibResultJsonFilepath;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        err_msg = "CALIB FAIL: Exception";
        FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
// if img != cv::Mat() then a recalibration is performed
GC_STATUS CalibExecutive::Load( const string jsonFilepath, const Mat &img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();
        if ( !fs::exists( jsonFilepath ) )
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << jsonFilepath << " does not exist";
            retVal = GC_ERR;
        }
        else
        {
            std::string jsonString;
            std::ifstream t( jsonFilepath );
            std::stringstream buffer;
            buffer << t.rdbuf();
            jsonString = buffer.str();
            // fs::load_string_file( jsonFilepath, jsonString );

            stringstream ss;
            ss << jsonString;
            // cout << endl << jsonString << endl;

            property_tree::ptree pt;
            property_tree::read_json( ss, pt );

            string calibTypeString = pt.get< string >( "calibType", "NotSet" );
            if ( calibTypeString == "BowTie" )
            {
                stopSign.clear();
                paramsCurrent.calibType = "BowTie";
                retVal = findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, Size( GC_IMAGE_SIZE_WIDTH, GC_IMAGE_SIZE_HEIGHT ) );
                if ( GC_OK == retVal )
                {
                    retVal = bowTie.Load( ss.str() );
                    if ( GC_OK == retVal )
                    {
                        Mat scratch( bowTie.GetModel().imgSize, CV_8UC1 );
                        retVal = findCalibGrid.SetMoveTargetROI( scratch, bowTie.MoveSearchROI( true ), bowTie.MoveSearchROI( false ) );
                    }
                }
            }
            else if ( calibTypeString == "StopSign" )
            {
                bowTie.clear();
                findCalibGrid.clear();
                paramsCurrent.calibType = "StopSign";
                paramsCurrent.calibResultJsonFilepath = jsonFilepath;
                retVal = stopSign.Load( ss.str() );
                if ( GC_OK == retVal )
                {
                    string controlJson = pt.get< string >( "control_json", "" );
                    if ( controlJson.empty() )
                    {
                        FILE_LOG( logERROR ) << "[CalibExecutive::Load] Could not retrieve calib control string from " << jsonFilepath;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        retVal = SetCalibFromJson( controlJson );
                        if ( GC_OK == retVal )
                        {
                            if (  img.empty() )
                            {
                                retVal = stopSign.CalcHomographies();
                            }
                            else
                            {
                                string err_msg;
                                retVal = CalibrateStopSign( img, controlJson, err_msg );
//                                if ( GC_OK != retVal )
//                                {
//                                    retVal = stopSign.Load( ss.str() );
//                                    if ( GC_OK == retVal )
//                                    {
//                                        cv::Point2d ptLft, ptRgt;
//                                        retVal = stopSign.SearchObj().FindMoveTargets( img, stopSign.TargetRoi(), ptLft, ptRgt );
//                                        if ( GC_OK == retVal )
//                                        {
//                                            retVal = stopSign.AdjustCalib( ptLft, ptRgt, stopSign.TargetRoi() );
//                                        }
//                                    }
//                                }
                            }
                        }
                    }
                }
            }
            else if ( "NotSet" == calibTypeString )
            {
                FILE_LOG( logERROR ) << "[CalibExecutive::Load] No calibration type specified in calibration file";
                retVal = GC_ERR;
            }
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
cv::Rect &CalibExecutive::TargetRoi()
{
    if ( "BowTie" == paramsCurrent.calibType )
    {
        return bowTie.TargetRoi();
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        return stopSign.TargetRoi();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::TargetRoi] No calibration type currently set";
    return nullRect;
}
GC_STATUS CalibExecutive::SetAdjustedSearchROI( std::vector< LineEnds > &searchLinesAdj )
{
    GC_STATUS retVal = GC_OK;
    if ( searchLinesAdj.empty() )
    {
        if ( "BowTie" == paramsCurrent.calibType )
        {
            bowTie.GetModel().waterlineSearchCornersAdj.clear();
        }
        else
        {
            stopSign.Model().waterlineSearchCornersAdj.clear();
        }
    }
    else
    {
        int last = searchLinesAdj.size() - 1;
        if ( "BowTie" == paramsCurrent.calibType )
        {
            bowTie.GetModel().waterlineSearchCornersAdj.clear();
            bowTie.GetModel().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].top );
            bowTie.GetModel().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].top );
            bowTie.GetModel().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].bot );
            bowTie.GetModel().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].bot );
        }
        else if ( "StopSign" == paramsCurrent.calibType )
        {
            stopSign.Model().waterlineSearchCornersAdj.clear();
            stopSign.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].top );
            stopSign.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].top );
            stopSign.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].bot );
            stopSign.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].bot );
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::SetAdjustedSearchROI] Invalid calib type";
            retVal = GC_ERR;
        }
    }
    return retVal;
}
std::vector< LineEnds > &CalibExecutive::SearchLines()
{
    if ( "BowTie" == paramsCurrent.calibType )
    {
        return bowTie.SearchLineSet();
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        return stopSign.SearchLineSet();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::SearchLines] No calibration type currently set";
    return nullSearchLines;
}
GC_STATUS CalibExecutive::GetMoveSearchROIs( Rect &rectLeft , Rect &rectRight )
{
    GC_STATUS retVal = GC_OK;

    if ( "BowTie" == paramsCurrent.calibType )
    {
        rectLeft = bowTie.MoveSearchROI( true );
        rectRight = bowTie.MoveSearchROI( true );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
GC_STATUS CalibExecutive::SetMoveSearchROIs( const cv::Mat img, const cv::Rect rectLeft, const cv::Rect rectRight )
{
    return findCalibGrid.SetMoveTargetROI( img, rectLeft, rectRight );
}
GC_STATUS CalibExecutive::FindMoveTargets( const Mat &img, FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;

    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = FindMoveTargetsBowTie( img, ptsFound );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = FindMoveTargetsStopSign( ptsFound );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
GC_STATUS CalibExecutive::FindMoveTargetsStopSign( FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;
    CalibModelSymbol model = stopSign.Model();
    if ( 8 == model.pixelPoints.size() )
    {
        ptsFound.lftPixel = model.pixelPoints[ 5 ];
        ptsFound.rgtPixel = model.pixelPoints[ 4 ];
        ptsFound.ctrPixel.x = ( ptsFound.lftPixel.x + ptsFound.rgtPixel.x ) / 2.0;
        ptsFound.ctrPixel.y = ( ptsFound.lftPixel.y + ptsFound.rgtPixel.y ) / 2.0;
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargetsStopSign] Valid calibration required";
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::FindMoveTargetsBowTie( const Mat &img, FindPointSet &ptsFound )
{
    GC_STATUS retVal = findCalibGrid.FindMoveTargets( img, bowTie.TargetRoi(), ptsFound.lftPixel, ptsFound.rgtPixel );
    if ( GC_OK == retVal )
    {
        ptsFound.ctrPixel.x = ( ptsFound.lftPixel.x + ptsFound.rgtPixel.x ) / 2.0;
        ptsFound.ctrPixel.y = ( ptsFound.lftPixel.y + ptsFound.rgtPixel.y ) / 2.0;
    }
    return retVal;
}
GC_STATUS CalibExecutive::MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = GC_OK;

    if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.MoveRefPoint( lftRefPt, rgtRefPt );
    }
    else if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = bowTie.MoveRefPoint( lftRefPt, rgtRefPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalculateRMSE( const cv::Mat &img, double &rmseEuclideanDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;
    rmseX = -9999999.0;
    rmseY = -9999999.0;
    rmseEuclideanDist = -9999999.0;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] The image must not be empty to calculate RMSE";
            retVal = GC_ERR;
        }
        else
        {

            vector< Point2d > reprojectedPts;
            Point2d projectedPt, reprojectedPt;
            for ( int row = 0; row < img.rows; ++row )
            {
                for ( int col = 0; col < img.cols; ++col )
                {
                    retVal = PixelToWorld( Point2d( col, row ), projectedPt );
                    if ( GC_OK == retVal )
                    {
                        retVal = WorldToPixel( projectedPt, reprojectedPt );
                        if ( GC_OK == retVal )
                        {
                            reprojectedPts.push_back( reprojectedPt );
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            if ( GC_OK == retVal )
            {
                int reprojectedIdx = 0;
                double diffSqrDist = 0.0;
                double diffSqrX = 0.0, diffSqrY = 0.0;
                for ( int row = 0; row < img.rows; ++row )
                {
                    for ( int col = 0; col < img.cols; ++col )
                    {
                        diffSqrX += pow( static_cast< double >( col ) - reprojectedPts[ reprojectedIdx ].x, 2 );
                        diffSqrY += pow( static_cast< double >( row ) - reprojectedPts[ reprojectedIdx ].y, 2 );
                        diffSqrDist += pow( Distance( Point2d( col, row ), reprojectedPts[ reprojectedIdx++ ] ), 2 );
                    }
                }

                rmseX = sqrt( diffSqrX );
                rmseY = sqrt( diffSqrY );
                rmseEuclideanDist = sqrt( diffSqrDist );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalculateRMSE( const std::vector< cv::Point2d > &foundPts, std::vector< cv::Point2d > &reprojectedPts,
                                         double &rmseEuclideanDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;
    rmseX = -9999999.0;
    rmseY = -9999999.0;
    rmseEuclideanDist = -9999999.0;

    try
    {
        if ( 2 > foundPts.size() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] There must be more than one point to calculate RMSE";
            retVal = GC_ERR;
        }
        else
        {
            reprojectedPts.clear();
            Point2d projectedPt, reprojectedPt;
            for ( size_t i = 0; i < foundPts.size(); ++i )
            {
                retVal = PixelToWorld( foundPts[ i ], projectedPt );
                if ( GC_OK == retVal )
                {
                    retVal = WorldToPixel( projectedPt, reprojectedPt );
                    if ( GC_OK == retVal )
                    {
                        reprojectedPts.push_back( reprojectedPt );
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                vector< double > euclidDists;
                double diffSqrDist = 0.0;
                double diffSqrX = 0.0, diffSqrY = 0.0;
                for ( size_t i = 0; i < foundPts.size(); ++i )
                {
                    diffSqrX += pow( foundPts[ i ].x - reprojectedPts[ i ].x, 2 );
                    diffSqrY += pow( foundPts[ i ].y - reprojectedPts[ i ].y, 2 );
                    diffSqrDist += pow( Distance( foundPts[ i ], reprojectedPts[ i ] ), 2 );
                }

                rmseX = sqrt( diffSqrX );
                rmseY = sqrt( diffSqrY );
                rmseEuclideanDist = sqrt( diffSqrDist );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] " << e.what();
    }

    return retVal;
}
GC_STATUS CalibExecutive::FormBowtieCalibJsonString(  const CalibJsonItems &items, std::string &json )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        json.clear();

        json = "{\"calibType\": \"BowTie\", ";
        json += "\"calibWorldPt_csv\": \"" + items.worldTargetPosition_csvFile + "\", ";
        json += "\"facetLength\": -1.0, ";
        json += "\"zeroOffset\": 0.0, ";
        json += "\"moveSearchROIGrowPercent\": " + std::to_string( items.moveROIGrowPercent ) + ", ";
        json += "\"drawCalib\": 0, ";
        json += "\"drawMoveSearchROIs\": 0, ";
        json += "\"drawWaterLineSearchROI\": 0, ";
        if ( items.useROI )
        {
            json += "\"targetRoi_x\": " + std::to_string( items.roi.x ) + ", ";
            json += "\"targetRoi_y\": " + std::to_string( items.roi.y ) + ", ";
            json += "\"targetRoi_width\": " + std::to_string( items.roi.width ) + ", ";
            json += "\"targetRoi_height\": " + std::to_string( items.roi.height ) + ", ";
        }
        else
        {
            json += "\"targetRoi_x\": -1, ";
            json += "\"targetRoi_y\": -1, ";
            json += "\"targetRoi_width\": -1, ";
            json += "\"targetRoi_height\": -1, ";
        }
        json += "\"searchPoly_lftTop_x\": " + std::to_string( items.lineSearchPoly.lftTop.x ) + ", ";
        json += "\"searchPoly_lftTop_y\": " + std::to_string( items.lineSearchPoly.lftTop.y ) + ", ";
        json += "\"searchPoly_rgtTop_x\": " + std::to_string( items.lineSearchPoly.rgtTop.x ) + ", ";
        json += "\"searchPoly_rgtTop_y\": " + std::to_string( items.lineSearchPoly.rgtTop.y ) + ", ";
        json += "\"searchPoly_lftBot_x\": " + std::to_string( items.lineSearchPoly.lftBot.x ) + ", ";
        json += "\"searchPoly_lftBot_y\": " + std::to_string( items.lineSearchPoly.lftBot.y ) + ", ";
        json += "\"searchPoly_rgtBot_x\": " + std::to_string( items.lineSearchPoly.rgtBot.x ) + ", ";
        json += "\"searchPoly_rgtBot_y\": " + std::to_string( items.lineSearchPoly.rgtBot.y ) + ", ";
        json += "\"calibResult_json\": \"" + items.calibVisionResult_json + "\"}";
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::FormBowtieCalibJsonString] " << e.what();
    }

    return retVal;
}
GC_STATUS CalibExecutive::FormStopsignCalibJsonString( const CalibJsonItems &items, std::string &json )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        json = "{\"calibType\": \"StopSign\", ";
        json += "\"calibWorldPt_csv\": \"" + items.worldTargetPosition_csvFile + "\", ";
        json += "\"facetLength\": " + std::to_string( items.facetLength ) + ", ";
        json += "\"zeroOffset\": " + std::to_string( items.zeroOffset ) + ", ";
        json += "\"moveSearchROIGrowPercent\": " + std::to_string( items.moveROIGrowPercent ) + ", ";
        json += "\"drawCalib\": 0, ";
        json += "\"drawMoveSearchROIs\": 0, ";
        json += "\"drawWaterLineSearchROI\": 0, ";
        if ( items.useROI )
        {
            json += "\"targetRoi_x\": " + std::to_string( items.roi.x ) + ", ";
            json += "\"targetRoi_y\": " + std::to_string( items.roi.y ) + ", ";
            json += "\"targetRoi_width\": " + std::to_string( items.roi.width ) + ", ";
            json += "\"targetRoi_height\": " + std::to_string( items.roi.height ) + ", ";
        }
        else
        {
            json += "\"targetRoi_x\": -1, ";
            json += "\"targetRoi_y\": -1, ";
            json += "\"targetRoi_width\": -1, ";
            json += "\"targetRoi_height\": -1, ";
        }
        json += "\"searchPoly_lftTop_x\": " + std::to_string( items.lineSearchPoly.lftTop.x ) + ", ";
        json += "\"searchPoly_lftTop_y\": " + std::to_string( items.lineSearchPoly.lftTop.y ) + ", ";
        json += "\"searchPoly_rgtTop_x\": " + std::to_string( items.lineSearchPoly.rgtTop.x ) + ", ";
        json += "\"searchPoly_rgtTop_y\": " + std::to_string( items.lineSearchPoly.rgtTop.y ) + ", ";
        json += "\"searchPoly_lftBot_x\": " + std::to_string( items.lineSearchPoly.lftBot.x ) + ", ";
        json += "\"searchPoly_lftBot_y\": " + std::to_string( items.lineSearchPoly.lftBot.y ) + ", ";
        json += "\"searchPoly_rgtBot_x\": " + std::to_string( items.lineSearchPoly.rgtBot.x ) + ", ";
        json += "\"searchPoly_rgtBot_y\": " + std::to_string( items.lineSearchPoly.rgtBot.y ) + ", ";
        json += "\"calibResult_json\": \"" + items.calibVisionResult_json + "\"}";
        // cout << json << endl;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::FormStopsignCalibJsonString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::GetCalibStopsignJsonItems( const std::string &jsonStr, CalibJsonItems &items )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        items.clear();

        stringstream ss;
        ss << jsonStr;
        // cout << ss.str() << endl;

        pt::ptree top_level;
        pt::json_parser::read_json( ss, top_level );

        items.facetLength = top_level.get< double >( "facetLength", -1.0 );
        items.worldTargetPosition_csvFile = top_level.get< int >( "moveSearchROIGrowPercent", 0 );
        items.calibVisionResult_json = top_level.get< string >( "calibResult_json", "" );
        items.zeroOffset = top_level.get< double >( "botLftPtToLft", -0.5 );
        items.useROI = top_level.get< int >( "useROI", 0 ) ? false : true;
        if ( items.useROI )
        {
            items.roi.x = top_level.get< int >( "targetRoi_x", -1 );
            items.roi.y = top_level.get< int >( "targetRoi_y", -1 );
            items.roi.width = top_level.get< int >( "targetRoi_width", -1 );
            items.roi.height = top_level.get< int >( "targetRoi_height", -1 );
        }
        else
        {
            items.roi = cv::Rect( -1, -1, -1, -1 );
        }
        items.lineSearchPoly.lftTop.x = top_level.get< int >( "searchPoly_lftTop_x", -1 );
        items.lineSearchPoly.lftTop.y = top_level.get< int >( "searchPoly_lftTop_y", -1 );
        items.lineSearchPoly.rgtTop.x = top_level.get< int >( "searchPoly_rgtTop_x", -1 );
        items.lineSearchPoly.rgtTop.y = top_level.get< int >( "searchPoly_rgtTop_y", -1 );
        items.lineSearchPoly.lftBot.x = top_level.get< int >( "searchPoly_lftBot_x", -1 );
        items.lineSearchPoly.lftBot.y = top_level.get< int >( "searchPoly_lftBot_y", -1 );
        items.lineSearchPoly.rgtBot.x = top_level.get< int >( "searchPoly_rgtBot_x", -1 );
        items.lineSearchPoly.rgtBot.y = top_level.get< int >( "searchPoly_rgtBot_y", -1 );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::FormStopsignCalibJsonString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
