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
    out << "\"facetLength\": " << params.facetLength << ", ";
    out << "\"zeroOffset\":" << params.zeroOffset << "," << endl;
    out << "\"botLftPtToLft\":" << params.botLftPtToLft << "," << endl;
    out << "\"botLftPtToTop\":" << params.botLftPtToTop << "," << endl;
    out << "\"botLftPtToRgt\":" << params.botLftPtToRgt << "," << endl;
    out << "\"botLftPtToBot\":" << params.botLftPtToBot << "," << endl;
    out << "\"calibResult_json\": \"" << params.calibResultJsonFilepath << "\", ";
    out << "\"drawCalibScale\": " << ( params.drawCalibScale ? 1 : 0 ) << ", ";
    out << "\"drawCalibGrid\": " << ( params.drawCalibGrid ? 1 : 0 ) << ", ";
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
    octagon.clear();
    calibFileJson.clear();
}
GC_STATUS CalibExecutive::GetTargetSearchROI( cv::Rect &rect )
{
    rect = octagon.TargetRoi();
    return GC_OK;
}
GC_STATUS CalibExecutive::GetCalibControlJson( std::string &calibJson )
{
    GC_STATUS retVal = GC_OK;
    if ( "Octagon" == GetCalibType() )
    {
        calibJson = octagon.ControlJson();
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::GetCalibControlJson] No calibration defined" ;
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::GetCalibParams( std::string &calibParams )
{
    GC_STATUS retVal = GC_OK;
    if ( "Octagon" == GetCalibType() )
    {
        retVal = octagon.GetCalibParams( calibParams );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Recalibrate] No calibration defined" ;
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::Calibrate(const Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                                    double &rmseDist, double &rmseX, double &rmseY,
                                    string &err_msg, const bool save, const bool drawAll )
{
    GC_STATUS retVal = Calibrate( img, jsonParams, rmseDist, rmseX, rmseY, err_msg, save );
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
        paramsCurrent.facetLength = top_level.get< double >( "facetLength", -1.0 );

        paramsCurrent.botLftPtToLft = top_level.get< double >( "botLftPtToLft", -0.5 );
        paramsCurrent.botLftPtToTop = top_level.get< double >( "botLftPtToTop", 1.0 );
        paramsCurrent.botLftPtToRgt = top_level.get< double >( "botLftPtToRgt", 1.5 );
        paramsCurrent.botLftPtToBot = top_level.get< double >( "botLftPtToBot", -3.0 );

        paramsCurrent.calibResultJsonFilepath = top_level.get< string >( "calibResult_json", "" );
        paramsCurrent.drawCalibScale = 1 == top_level.get< int >( "drawCalibScale", 0 );
        paramsCurrent.drawCalibGrid = 1 == top_level.get< int >( "drawCalibGrid", 0 );
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

        if ( "Octagon" == paramsCurrent.calibType )
        {
            octagon.Model().controlJson = jsonParams;
            octagon.Model().facetLength = paramsCurrent.facetLength;
            octagon.Model().zeroOffset = top_level.get< double >( "zeroOffset", 0.0 );
            octagon.Model().targetSearchRegion = paramsCurrent.targetSearchROI;
            octagon.Model().waterlineSearchCorners.clear();

            octagon.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_lftTop );
            octagon.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_rgtTop );
            octagon.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_lftBot );
            octagon.Model().waterlineSearchCorners.push_back( paramsCurrent.lineSearch_rgtBot );
            sort( octagon.Model().waterlineSearchCorners.begin(), octagon.Model().waterlineSearchCorners.end(),
                  []( Point const &a, Point const &b ) { return ( a.y < b.y ); } );
            if ( octagon.Model().waterlineSearchCorners[ 0 ].x > octagon.Model().waterlineSearchCorners[ 1 ].x )
            {
                Point ptTemp = octagon.Model().waterlineSearchCorners[ 0 ];
                octagon.Model().waterlineSearchCorners[ 0 ] = octagon.Model().waterlineSearchCorners[ 1 ];
                octagon.Model().waterlineSearchCorners[ 1 ] = ptTemp;
            }
            if ( octagon.Model().waterlineSearchCorners[ 2 ].x > octagon.Model().waterlineSearchCorners[ 3 ].x )
            {
                Point ptTemp = octagon.Model().waterlineSearchCorners[ 2 ];
                octagon.Model().waterlineSearchCorners[ 2 ] = octagon.Model().waterlineSearchCorners[ 3 ];
                octagon.Model().waterlineSearchCorners[ 3 ] = ptTemp;
            }
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::SetCalibFromJson] Invalid calibration type=" <<
                                    ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
            retVal = GC_ERR;
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::SetCalibFromJson] " << diagnostic_information( e );
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
GC_STATUS CalibExecutive::SetCalibModel( CalibModelOctagon newModel )
{
    GC_STATUS retVal = octagon.SetCalibModel( newModel );
    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const cv::Mat &img, const std::string jsonParams,
                                     double &rmseDist, double &rmseX, double &rmseY,
                                     string &err_msg, const bool save )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        string jsonParamsWhich = jsonParams;
        if ( jsonParamsWhich.empty() )
        {
            if ( "Octagon" == paramsCurrent.calibType )
            {
                if ( !octagon.Model().controlJson.empty() )
                {
                    jsonParamsWhich = octagon.Model().controlJson;
                }
                else
                {
                    err_msg = "CALIB FAIL: No available octagon calibration control string";
                    FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] No available octagon calibration control string";
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
                if ( "Octagon" == paramsCurrent.calibType )
                {
                    retVal = CalibrateOctagon( imgFixed, jsonParamsWhich, err_msg );
                    if ( GC_OK == retVal )
                    {
                        retVal = octagon.GetSearchRegionBoundingRect( searchBB );
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
                        FILE_LOG( logWARNING ) << "[CalibExecutive::Calibrate] Could not calculate RMSE";
                        retVal = GC_OK;
                    }
                    else if ( save )
                    {
                        retVal = octagon.Save( paramsCurrent.calibResultJsonFilepath );
                        if ( GC_OK != retVal )
                        {
                            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Could not calculate RMSE";
                        }
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
GC_STATUS CalibExecutive::AdjustOctagonForRotation( const Size imgSize, const FindPointSet &calcLinePts, double &offsetAngle )
{
    GC_STATUS retVal = octagon.AdjustOctagonForRotation( imgSize, calcLinePts, offsetAngle );
    return retVal;
}
GC_STATUS CalibExecutive::DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg )
{
    {
        GC_STATUS retVal = GC_OK;
        if ( "Octagon" == paramsCurrent.calibType )
        {
            retVal = octagon.DrawAssocPts( img, overlay, err_msg );
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
        retVal = DrawOverlay( matIn, imgMatOut, false, true, true, true );
    }
    else
    {
        retVal = DrawOverlay( matIn, imgMatOut, paramsCurrent.drawCalibScale, paramsCurrent.drawCalibGrid,
                                        paramsCurrent.drawWaterLineSearchROI, paramsCurrent.drawTargetSearchROI );
    }
    return retVal;
}
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale,
                                       const bool drawCalibGrid, const bool drawSearchROI, const bool drawTargetROI )
{
    GC_STATUS retVal = GC_OK;
    if ( "Octagon" == paramsCurrent.calibType )
    {
        retVal = octagon.DrawOverlay( matIn, imgMatOut, drawCalibScale, drawCalibGrid, drawSearchROI, drawTargetROI );
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
    if ( "Octagon" == paramsCurrent.calibType )
    {
        retVal = octagon.PixelToWorld( pixelPt, worldPt );
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
    if ( "Octagon" == paramsCurrent.calibType )
    {
        retVal = octagon.WorldToPixel( worldPt, pixelPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::WorldToPixel] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibOctagon::GetSearchRegionBoundingRect( cv::Rect &rect )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( model.searchLineSet.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibOctagon::GetSearchRegionBoundingRect] System not calibrated";
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
        FILE_LOG( logERROR ) << "[CalibOctagon::GetSearchRegionBoundingRect] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::CalibSaveOctagon( const string jsonFilepath )
{
    paramsCurrent.calibResultJsonFilepath = jsonFilepath;
    octagon.Model().oldPixelPoints.clear();
    GC_STATUS retVal = octagon.Save( paramsCurrent.calibResultJsonFilepath );
    return retVal;
}
GC_STATUS CalibExecutive::CalibrateOctagon( const cv::Mat &img, const string &controlJson, string &err_msg )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = octagon.Calibrate( img, controlJson, err_msg );
        if ( GC_OK != retVal )
        {
            Mat matIn;

            // GaussianBlur( matIn, matIn, Size( 5, 5 ), 1.0 );
            medianBlur( img, matIn, 7 );

            Mat kern = getStructuringElement( MORPH_ELLIPSE, Size( 5, 5 ) );
            erode( matIn, matIn, kern, Point( -1, -1 ), 1 );
            retVal = octagon.Calibrate( matIn, controlJson, err_msg );
        }
    }
    catch( Exception &e )
    {
        err_msg = "CALIB FAIL: Exception";
        FILE_LOG( logERROR ) << "[VisApp::CalibrateOctagon] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::LoadFromJsonString()
{
    string jsonString = calibFileJson;
    GC_STATUS retVal = LoadFromJsonString( jsonString );
    return retVal;
}

GC_STATUS CalibExecutive::LoadFromJsonString( const std::string jsonString, const std::string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();
        stringstream ss;
        ss << jsonString;
        // cout << endl << jsonString << endl;

        property_tree::ptree pt;
        property_tree::read_json( ss, pt );

        string calibTypeString = pt.get< string >( "calibType", "NotSet" );
        if ( calibTypeString == "Octagon" )
        {
            paramsCurrent.calibType = "Octagon";
            if ( !jsonFilepath.empty() )
                paramsCurrent.calibResultJsonFilepath = jsonFilepath;

            retVal = octagon.Load( ss.str() );
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
                        retVal = octagon.CalcHomographies();
                    }
                }
            }
        }
        else if ( "NotSet" == calibTypeString )
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::LoadFromJsonString] No calibration type specified in calibration file";
            retVal = GC_ERR;
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::Load( const string jsonFilepath )
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

            retVal = LoadFromJsonString( jsonString, jsonFilepath );
            calibFileJson = GC_OK == retVal ? jsonString : "";
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
    if ( "Octagon" == paramsCurrent.calibType )
    {
        return octagon.TargetRoi();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::TargetRoi] No calibration type currently set";
    return nullRect;
}
GC_STATUS CalibExecutive::SetAdjustedSearchROI( std::vector< LineEnds > &searchLinesAdj )
{
    GC_STATUS retVal = GC_OK;
    if ( searchLinesAdj.empty() )
    {
        octagon.Model().waterlineSearchCornersAdj.clear();
    }
    else
    {
        int last = searchLinesAdj.size() - 1;
        if ( "Octagon" == paramsCurrent.calibType )
        {
            octagon.Model().waterlineSearchCornersAdj.clear();
            octagon.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].top );
            octagon.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].top );
            octagon.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ last ].bot );
            octagon.Model().waterlineSearchCornersAdj.push_back( searchLinesAdj[ 0 ].bot );
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
    if ( "Octagon" == paramsCurrent.calibType )
    {
        return octagon.SearchLineSet();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::SearchLines] No calibration type currently set";
    return nullSearchLines;
}
GC_STATUS CalibExecutive::FindMoveTargets( FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;

    if ( "Octagon" == paramsCurrent.calibType )
    {
        retVal = MoveFountPoint( ptsFound );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
GC_STATUS CalibExecutive::MoveFountPoint( FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 8 == CalibModel().pixelPoints.size() )
        {
            std::vector< Point2d > foundCalPts = CalibModel().pixelPoints;
            ptsFound.lftPixel = foundCalPts[ 5 ];
            ptsFound.rgtPixel = foundCalPts[ 4 ];
            ptsFound.ctrPixel.x = ( ptsFound.lftPixel.x + ptsFound.rgtPixel.x ) / 2.0;
            ptsFound.ctrPixel.y = ( ptsFound.lftPixel.y + ptsFound.rgtPixel.y ) / 2.0;
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::FindMoveTargetsOctagon] Valid calibration required";
            retVal = GC_ERR;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::FindMoveTargetsOctagon] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = GC_OK;

    if ( "Octagon" == paramsCurrent.calibType )
    {
        retVal = octagon.MoveRefPoint( lftRefPt, rgtRefPt );
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
            FILE_LOG( logERROR ) << "[CalibExecutive::CalculateRMSE] The image must not be empty to calculate RMSE";
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
        FILE_LOG( logERROR ) << "[CalibExecutive::CalculateRMSE] " << e.what();
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
            FILE_LOG( logERROR ) << "[CalibExecutive::CalculateRMSE] There must be more than one point to calculate RMSE";
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
        FILE_LOG( logERROR ) << "[CalibExecutive::CalculateRMSE] " << e.what();
    }

    return retVal;
}
GC_STATUS CalibExecutive::FormOctagonCalibJsonString( const CalibJsonItems &items, std::string &json )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        json = "{\"calibType\": \"Octagon\", ";
        json += "\"facetLength\": " + std::to_string( items.facetLength ) + ", ";
        json += "\"zeroOffset\": " + std::to_string( items.zeroOffset ) + ", ";
        json += "\"drawCalib\": 0, ";
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
        FILE_LOG( logERROR ) << "[CalibExecutive::FormOctagonCalibJsonString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::GetCalibOctagonJsonItems( const std::string &jsonStr, CalibJsonItems &items )
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
        FILE_LOG( logERROR ) << "[CalibExecutive::GetCalibOctagonJsonItems] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
