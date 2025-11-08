/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "log.h"
#include "visapp.h"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "timestampconvert.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = std::filesystem;
namespace pt = property_tree;

#ifdef DEBUG_BOWTIE_FIND
#undef DEBUG_BOWTIE_FIND
#ifdef _WIN32
static const char DEBUG_FOLDER[] = "c:/gaugecam/";
#else
static const char DEBUG_FOLDER[] = "/var/tmp/gaugecam/";
#endif
#endif

namespace gc
{

VisApp::VisApp() :
    m_calibFilepath( "" )
{
    try
    {
#ifdef _WIN32
        if ( !fs::exists( "c:/gaugecam/" ) )
        {
            fs::create_directories( "c:/gaugecam/" );
        }
#else
        if ( !fs::exists( "/var/tmp/gaugecam/" ) )
        {
            fs::create_directories( "/var/tmp/gaugecam/" );
        }
#endif
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::VisApp] Creating debug folder" << diagnostic_information( e );
    }
}
GC_STATUS VisApp::GetTempCacheResults( const std::string jsonFilepath, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        result.clear();
        if ( !fs::exists( jsonFilepath ) )
        {
            FILE_LOG( logERROR ) << "[VisApp::GetTempCacheResults] " << jsonFilepath << " does not exist";
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

            string status = pt.get< string >( "STATUS", "FAILURE" );
            if ( "SUCCESS" == status )
            {
                result.findSuccess = true;
                result.timestamp = pt.get< string >( "timestamp", "1955-09-24T12:00:00" );
                result.waterLevelAdjusted.x = pt.get< double >( "world_line_center_x", -9999999.999 );
                result.waterLevelAdjusted.y = pt.get< double >( "world_line_center_y", -9999999.999 );
            }
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::GetTempCacheResults] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::CalibLoad( const std::string calibJson )
{
    GC_STATUS retVal = m_calibExec.Load( calibJson );
    return retVal;
}
GC_STATUS VisApp::CalibSave( const std::string jsonPath )
{
    GC_STATUS retVal = m_calibExec.CalibSaveOctagon( jsonPath );
    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string jsonControl, const string resultImgPath,
                             double &rmseDist, double &rmseX, double &rmseY, string &err_msg, const bool save )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat img = imread( imgFilepath, IMREAD_COLOR );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::Calibrate] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            retVal = m_calibExec.Calibrate( img, jsonControl, rmseDist, rmseX, rmseY, err_msg, save );
            if ( GC_OK == retVal )
            {
                Mat imgOut;
                retVal = m_calibExec.DrawOverlay( img, imgOut );
                if ( GC_OK == retVal )
                {
                    bool bRet = imwrite( resultImgPath, imgOut );
                    if ( !bRet )
                    {
                        err_msg = "CALIB FAIL: Could not write calibration result image";
                        FILE_LOG( logERROR ) << "[VisApp::Calibrate] Could not write result image " << resultImgPath;
                        retVal = GC_ERR;
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        err_msg = "CALIB FAIL: Exception";
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string jsonControl, double &rmseDist, double &rmseX, double &rmseY, string &err_msg, const bool save )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat img = imread( imgFilepath, IMREAD_COLOR );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::Calibrate] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            retVal = Calibrate( img, jsonControl, rmseDist, rmseX, rmseY, err_msg, save );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::Calibrate( const Mat &img, const string jsonControl, double &rmseDist, double &rmseX, double &rmseY, string &err_msg, const bool save )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat scratch;
        if ( 1 == img.channels() )
        {
            scratch = img;
        }
        else
        {
            cvtColor( img, scratch, COLOR_BGR2GRAY );
        }
        retVal = m_calibExec.Calibrate( scratch, jsonControl, rmseDist, rmseX, rmseY, err_msg, save );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::SetMinMaxFindLineAngles( const double minAngle, const double maxAngle )
{
    GC_STATUS retVal = m_findLine.SetLineFindAngleBounds( minAngle, maxAngle );
    return retVal;
}

GC_STATUS VisApp::GetImageTimestamp( const std::string filepath, std::string &timestamp )
{
    GC_STATUS retVal = m_metaData.GetExifData( filepath, "DateTimeOriginal", timestamp );
    if ( GC_OK != retVal )
    {
        GC_STATUS retVal = m_metaData.GetExifData( filepath, "CaptureTime", timestamp );
        if ( GC_OK != retVal )
        {
            FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could not retrieve exif timestamp from " << filepath;
        }
    }
    return retVal;
}
GC_STATUS VisApp::GetIllumination( const std::string filepath, std::string &illum_state )
{
    GC_STATUS retVal = m_metaData.GetExifData( filepath, "Illumination", illum_state );
    if ( GC_OK != retVal )
    {
        retVal = m_metaData.GetExifData( filepath, "Flash", illum_state );
        if ( GC_OK != retVal )
        {
            illum_state = "N/A";
        }
    }
    return retVal;
}
GC_STATUS VisApp::GetImageData( const std::string filepath, std::string &data )
{
    GC_STATUS retVal = m_metaData.GetImageData( filepath, data );
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could  not retrieve exif image data from " << filepath;
    }
    return retVal;
}
GC_STATUS VisApp::GetImageData( const std::string filepath, ExifFeatures &exifFeat )
{
    GC_STATUS retVal = m_metaData.GetImageData( filepath, exifFeat );
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could  not retrieve exif image data from " << filepath;
    }
    return retVal;
}
GC_STATUS VisApp::AdjustSearchAreaForMovement( const std::vector< LineEnds > &searchLines, std::vector< LineEnds > &searchLinesAdj,
                                               const cv::Point2d searchROIcenter, const cv::Point2d octoCenter )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cv::Point2d offsets = searchROIcenter - octoCenter;
        if ( searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::AdjustSearchAreaForMovement] No lines in search line vector";
            retVal = GC_ERR;
        }
        else if ( 0.0 == offsets.x && 0.0 == offsets.y )
        {
            searchLinesAdj = searchLines;
        }
        else
        {
            searchLinesAdj.clear();
            for ( size_t i = 0; i < searchLines.size(); ++i )
            {
                searchLinesAdj.push_back( LineEnds( searchLines[ i ].top - Point( cvRound( offsets.x ), cvRound( offsets.y ) ),
                                                    searchLines[ i ].bot - Point( cvRound( offsets.x ), cvRound( offsets.y ) ) ) );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::AdjustSearchAreaForMovement] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::CalcFindLine( const Mat &img, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;
    if ( !m_calibExec.isCalibrated() )
    {
        result.msgs.push_back( "Find line failure: System not calibrated" );
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            char buffer[ 256 ];
            snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
            result.msgs.push_back( buffer );

            vector< LineEnds > searchLinesAdj;
            if ( GC_OK == retVal )
            {
                string err_msg;
                double rmseDist, rmseX, rmseY;
                retVal = m_calibExec.Calibrate( img, m_calibExec.CalibModel().controlJson, rmseDist, rmseX, rmseY, err_msg );
                if ( GC_OK != retVal )
                {
                    result.msgs.push_back( "Octagon calibration failed" );
                }
                else
                {
                    result.octoCenter = m_calibExec.CalibModel().OctoCenterPixel;
                    Rect roi = m_calibExec.TargetRoi();
                    Point2d searchROICenter( ( roi.x + roi.width / 2.0 ), ( roi.y + roi.height / 2.0 ) );
                    retVal = AdjustSearchAreaForMovement( m_calibExec.SearchLines(), searchLinesAdj, searchROICenter, result.octoCenter );
                    if ( GC_OK == retVal )
                    {
                        retVal = m_calibExec.SetAdjustedSearchROI( searchLinesAdj );
                    }
                }
            }
            else
            {
                result.findSuccess = false;
                result.msgs.push_back( "Invalid target type for line find" );
                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Invalid target typefor line find";
                retVal = GC_ERR;
            }

            if ( GC_OK == retVal )
            {
                retVal = m_findLine.Find( img, searchLinesAdj, result );
                if ( GC_OK != retVal )
                {
                    m_findLineResult = result;
                    result.msgs.push_back( "Could not perform find with provided image and calibration" );
                    FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not perform find with provided image and calibration";
                    retVal = GC_ERR;
                }
                else
                {
                    if ( "Octagon" == m_calibExec.GetCalibType() )
                    {
                        retVal = m_calibExec.AdjustOctagonForRotation( img.size(), result.calcLinePts, result.symbolToWaterLineAngle );
                    }
                    if ( GC_OK == retVal )
                    {
                        result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );

                        retVal = PixelToWorld( result.calcLinePts );
                        if ( GC_OK != retVal )
                        {
                            result.msgs.push_back( "Could not calculate world coordinates for found line points" );
                        }
                        else
                        {
                            // snprintf( buffer, 256, "%s/waterline angle diff: %.3f", m_calibExec.GetCalibType().c_str(), result.symbolToWaterLineAngle );
                            snprintf( buffer, 256, "CalibType: %s", m_calibExec.GetCalibType().c_str() );
                            result.msgs.push_back( buffer );

                            result.calcLinePts.angleWorld = atan2( result.calcLinePts.rgtWorld.y - result.calcLinePts.lftWorld.y,
                                                                   result.calcLinePts.rgtWorld.x - result.calcLinePts.lftWorld.x ) * ( 180.0 / CV_PI );
                            snprintf( buffer, 256, "Angle: %.3f", result.calcLinePts.angleWorld );
                            result.msgs.push_back( buffer );

                            snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                            result.msgs.push_back( buffer );

                            Point2d reprojectPt;
                            retVal = WorldToPixel( result.calcLinePts.ctrWorld, reprojectPt );
                            if ( GC_OK == retVal )
                            {
                                result.calibReprojectOffset_x = result.calcLinePts.ctrPixel.x - reprojectPt.x;
                                result.calibReprojectOffset_y = result.calcLinePts.ctrPixel.y - reprojectPt.y;
                                double dist = sqrt( pow( result.calcLinePts.ctrPixel.x - reprojectPt.x, 2 ) +
                                                    pow( result.calcLinePts.ctrPixel.y - reprojectPt.y, 2 ) );
                                result.calibReprojectOffset_dist = dist;
                            }
                            else
                            {
                                result.calibReprojectOffset_x = -9999999.0;
                                result.calibReprojectOffset_y = -9999999.0;
                                result.calibReprojectOffset_dist = -9999999.0;;
                            }
                        }
                    }
                }
            }
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::FindLine] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params )
{
    GC_STATUS retVal = CalcLine( params, m_findLineResult );
    return retVal;
}
GC_STATUS VisApp::CalcLine( const Mat &img, const string timestamp, const bool isOctagon )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        FindLineResult result;
        if ( img.empty() )
        {
            result.findSuccess = false;
            FILE_LOG( logERROR ) << "[VisApp::CalcLine] Empty image";
            retVal = GC_ERR;
        }
        else
        {
            result.timestamp = timestamp;

            if ( isOctagon )
            {
                string err_msg;
                double rmseDist, rmseX, rmseY;
                retVal = m_calibExec.Calibrate( img, "", rmseDist, rmseX, rmseY, err_msg );
            }
            if ( GC_OK == retVal )
            {
                retVal = CalcFindLine( img, result );
                if ( GC_OK != retVal )
                {
                    result.findSuccess = false;
                    FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                    retVal = GC_ERR;
                }
                else
                {
                    result.waterLevelAdjusted.y += 0.0;
                }
            }
        }
        m_findLineResult = result;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLine] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::SearchLineRoiResultToJsonString( const bool findSuccess, const cv::Rect roi, const std::vector< cv::Point > maskPoly,
                                                   const std::vector< cv::Point > waterPoly, std::string &resultJson )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        stringstream ss;
        if ( 4 != maskPoly.size() )
        {
            ss << "{\"STATUS\":\"FAILURE -- Invalid mask polyline point count\"}";
        }
        else if ( 4 != waterPoly.size() )
        {
            ss << "{\"STATUS\":\"FAILURE -- Invalid water polyline point count\"}";
        }
        else
        {
            resultJson.clear();
            ss << "{\"STATUS\": " << ( findSuccess ? "\"SUCCESS\"," : "\"FAILURE\"," );
            ss << "\"ROI\":{";
            ss << "\"left\":" << roi.x << ", \"top\":" << roi.y << ",";
            ss << "\"width\":" << roi.width << ", \"height\":" << roi.height << "},";
            ss << "\"mask_poly_points\":[";
            ss << "{\"x\":" << maskPoly[0].x << ",\"y\":" << maskPoly[0].y << "},";
            ss << "{\"x\":" << maskPoly[1].x << ",\"y\":" << maskPoly[1].y << "},";
            ss << "{\"x\":" << maskPoly[2].x << ",\"y\":" << maskPoly[2].y << "},";
            ss << "{\"x\":" << maskPoly[3].x << ",\"y\":" << maskPoly[3].y << "}],";
            ss << "\"water_poly_points\":[";
            ss << "{\"x\":" << waterPoly[0].x << ",\"y\":" << waterPoly[0].y << "},";
            ss << "{\"x\":" << waterPoly[1].x << ",\"y\":" << waterPoly[1].y << "},";
            ss << "{\"x\":" << waterPoly[2].x << ",\"y\":" << waterPoly[2].y << "},";
            ss << "{\"x\":" << waterPoly[3].x << ",\"y\":" << waterPoly[3].y << "}]}";
        }
        resultJson = ss.str();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::SearchLineRoiResultToJsonString] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::ResultToJsonString( const FindLineResult result, const FindLineParams params, std::string &resultJson )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        stringstream ss;
        resultJson.clear();
        ss << "{\"STATUS\": " << ( result.findSuccess ? "\"SUCCESS\"," : "\"FAILURE\"," );
        ss << "\"image_path\": \"" << params.imagePath << "\",";
        ss << "\"calib_path\": \"" << params.calibFilepath << "\",";
        ss << "\"result_path\": \"" << params.resultImagePath << "\",";
        ss << "\"timestamp_type\": \"" << params.timeStampType << "\",";
        ss << "\"timestamp_format\": \"" << params.timeStampFormat << "\",";
        ss << "\"timestamp_start_pos\": " << params.timeStampStartPos << ",";
        ss << "\"timestamp_length\": " << params.timeStampStartPos << ",";
        ss << "\"timestamp\": \"" << result.timestamp << "\",";
        if ( GC_OK == retVal )
        {
            ss << "\"searchROICenter_x\": " << result.octoToSearchROIOffsetPixel << ",";
            ss << "\"searchROICenter_y\": " << result.octoToSearchROIOffsetWorld << ",";
            ss << "\"octagonCenter_x\": " << result.octoCenter.x << ",";
            ss << "\"octagonCenter_y\": " << result.octoCenter.y << ",";

            ss << "\"pixel_line_left_x\": " << result.calcLinePts.lftPixel.x << ",";
            ss << "\"pixel_line_left_y\": " << result.calcLinePts.lftPixel.y << ",";
            ss << "\"pixel_line_center_x\": " << result.calcLinePts.ctrPixel.x << ",";
            ss << "\"pixel_line_center_y\": " << result.calcLinePts.ctrPixel.y << ",";
            ss << "\"pixel_line_right_x\": " << result.calcLinePts.rgtPixel.x << ",";
            ss << "\"pixel_line_right_y\": " << result.calcLinePts.rgtPixel.y << ",";
            ss << "\"pixel_line_angle\": " << result.calcLinePts.anglePixel << ",";

            ss << "\"world_line_left_x\": " << result.calcLinePts.lftWorld.x << ",";
            ss << "\"world_line_left_y\": " << result.calcLinePts.lftWorld.y << ",";
            ss << "\"world_line_center_x\": " << result.calcLinePts.ctrWorld.x << ",";
            ss << "\"world_line_center_y\": " << result.calcLinePts.ctrWorld.y << ",";
            ss << "\"world_line_right_x\": " << result.calcLinePts.rgtWorld.x << ",";
            ss << "\"world_line_right_y\": " << result.calcLinePts.rgtWorld.y << ",";
            ss << "\"world_line_angle\": " << result.calcLinePts.angleWorld << ",";

            ss << "\"found_pts\": [";
            for ( size_t i = 0; i < result.foundPoints.size(); ++i )
            {
                ss << "{\"x\": " << result.foundPoints[ i ].x << ",";
                ss <<  "\"y\": " << result.foundPoints[ i ].y << "}";
                if ( result.foundPoints.size() - 1 != i )
                    ss << ",";
            }
            ss << "],";
        }
        ss << "\"messages\": [";
        for ( size_t i = 0; i < result.msgs.size(); ++i )
        {
            ss << "\"" << result.msgs[ i ] << "\"";
            if ( result.msgs.size() - 1 != i )
                ss << ",";
        }
        ss << "]";
        ss << "}";
        resultJson = ss.str();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::ResultToJsonString] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << params.imagePath << " calib=" << params.calibFilepath;
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params, FindLineResult &result, string &resultJson )
{
    GC_STATUS retVal = CalcLine( params, result );
    try
    {
        retVal = ResultToJsonString( result, params, resultJson );
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLine] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << params.imagePath << " calib=" << params.calibFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        result.clear();
        m_findLineResult.clear();
        cv::Mat img = imread( params.imagePath, IMREAD_COLOR );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLine] Empty image=" << params.imagePath ;
            retVal = GC_ERR;
        }
        else
        {
            if ( FROM_FILENAME == params.timeStampType )
            {
                retVal = GcTimestampConvert::GetTimestampFromString( fs::path( params.imagePath ).filename().string(),
                                                                     params.timeStampStartPos, params.timeStampFormat, result.timestamp );
            }
            else if ( FROM_EXIF == params.timeStampType )
            {
                string timestampTemp;
                retVal = GetImageTimestamp( params.imagePath, timestampTemp );
                if ( GC_OK == retVal )
                {
                    retVal = GcTimestampConvert::GetTimestampFromString( timestampTemp, params.timeStampStartPos,
                                                                         params.timeStampFormat, result.timestamp );
                }
            }
            else
            {
                FILE_LOG( logERROR ) << "Invalid timestamp type";
                retVal = GC_ERR;
            }

            if ( GC_OK != retVal )
            {
                result.msgs.push_back( "Timestamp failure. Check source, format, and start position of timestamp" );
            }
            else
            {
                if ( GC_OK == retVal )
                {
                    if ( params.isOctagonCalib || ( params.calibFilepath != m_calibFilepath && !params.isOctagonCalib ) )
                    {
                        // retVal = m_calibExec.isCalibrated() ? m_calibExec.LoadFromJsonString() : m_calibExec.Load( params.calibFilepath );
                        retVal = m_calibExec.Load( params.calibFilepath );
                        if ( GC_OK != retVal )
                        {
                            result.calibSuccess = false;
                            result.msgs.push_back( "Could not load calibration" );
                            FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not load calibration=" << params.calibFilepath ;
                            retVal = GC_ERR;
                        }
                    }
                }
                if ( GC_OK == retVal)
                {
                    retVal = GetIllumination( params.imagePath, result.illum_state );
                    m_calibFilepath = params.calibFilepath;

                    retVal = CalcFindLine( img, result );
                    if ( GC_OK == retVal )
                    {

                    }
                    else
                    {
                        result.findSuccess = false;
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                        retVal = GC_ERR;
                    }
                }
                m_findLineResult = result;
                if ( !params.resultCSVPath.empty() )
                {
                    WriteFindlineResultToCSV( params.resultCSVPath, params.imagePath, result );
                }

                if ( !params.resultImagePath.empty() )
                {
                    string resultJson;
                    retVal = ResultToJsonString( result, params, resultJson );
                    if ( GC_OK != retVal )
                    {
                        resultJson = "{\"STATUS\": \"FAILURE -- Could not retrive result json string\"}";
                    }
                    else
                    {
                        Mat color;
                        GC_STATUS retVal1 = DrawLineFindOverlay( img, color, result );
                        if ( GC_OK == retVal1 )
                        {
                            bool isOk = imwrite( params.resultImagePath, color );
                            if ( isOk )
                            {
                                retVal = m_metaData.WriteToImageDescription( params.resultImagePath, resultJson );
                            }
                            else
                            {
                                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not write result image to " << params.resultImagePath;
                                retVal1 = GC_ERR;
                            }
                        }
                    }
                }

                if ( !params.lineSearchROIFolder.empty() )
                {
                    string searchROIPath = params.lineSearchROIFolder;
                    if ( '/' != searchROIPath[ searchROIPath.size() - 1 ] )
                    {
                        searchROIPath += '/';
                    }

                    string outputROIPath = searchROIPath + fs::path( params.imagePath ).stem().string() + "_search_line_roi_and_mask.png";
                    retVal = SaveLineFindSearchRoi( img, outputROIPath, result );
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLine] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << params.imagePath << " calib=" << params.calibFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::LineIntersection( const LineEnds line1, const LineEnds line2, cv::Point2d &r )
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
            FILE_LOG( logERROR ) << "[OctagonSearch::LineIntersection] Lines are parallel";
            return GC_ERR;
        }

        double t1 = ( x.x * d2.y - x.y * d2.x ) / cross;
        r = Point2d( line1.top ) + d1 * t1;
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::LineIntersection] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisApp::SaveLineFindSearchRoi( const cv::Mat &img, const std::string resultImgPath, const FindLineResult result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        vector< Point > searchRoiPoly = m_calibExec.CalibModel().waterlineSearchCorners;
        Rect roi( Point( std::min( searchRoiPoly[ 0 ].x, searchRoiPoly[ 2 ].x ),
                         std::min( searchRoiPoly[ 0 ].y, searchRoiPoly[ 1 ].y ) ),
                  Point( std::max( searchRoiPoly[ 1 ].x, searchRoiPoly[ 3 ].x ),
                         std::max( searchRoiPoly[ 2 ].y, searchRoiPoly[ 3 ].y ) ) );

        Point2d lftWtrPt, rgtWtrPt;
        retVal = LineIntersection( LineEnds( searchRoiPoly[ 0 ], searchRoiPoly[ 2 ] ),
                                   LineEnds( result.calcLinePts.lftPixel, result.calcLinePts.rgtPixel ), lftWtrPt );
        if ( GC_OK == retVal )
        {
            retVal = LineIntersection( LineEnds( searchRoiPoly[ 1 ], searchRoiPoly[ 3 ] ),
                                       LineEnds( result.calcLinePts.lftPixel, result.calcLinePts.rgtPixel ), rgtWtrPt );
            if ( GC_OK == retVal )
            {
                Mat scratch1 = Mat::zeros( img.size(), CV_8UC1 );

                vector< Point > maskPoly;
                maskPoly.push_back( searchRoiPoly[ 0 ] );
                maskPoly.push_back( searchRoiPoly[ 1 ] );
                maskPoly.push_back( searchRoiPoly[ 3 ] );
                maskPoly.push_back( searchRoiPoly[ 2 ] );
                vector< vector< Point > > wholePoly{ maskPoly };
                cv::fillPoly( scratch1, wholePoly, Scalar( 128 ) );

                maskPoly[ 0 ] = lftWtrPt;
                maskPoly[ 1 ] = rgtWtrPt;
                vector< vector< Point > > waterPoly{ maskPoly };
                Mat scratch2 = Mat::zeros( img.size(), CV_8UC1 );
                cv::fillPoly( scratch2, waterPoly, Scalar( 64 ) );
                scratch1 += scratch2;

                Mat outputImg = Mat::zeros( Size( roi.width << 1, roi.height ), CV_8UC3 );
                if ( CV_8UC3 == img.type() )
                {
                    img( roi ).copyTo( outputImg( Rect( 0, 0, roi.width, roi.height ) ) );
                }
                else
                {
                    cvtColor( img( roi ), outputImg( Rect( 0, 0, roi.width, roi.height ) ), COLOR_GRAY2BGR );
                }
                cvtColor( scratch1( roi ), outputImg( Rect( roi.width, 0, roi.width, roi.height ) ), COLOR_GRAY2BGR );

                bool bRet = imwrite( resultImgPath, outputImg );
                if ( bRet )
                {
                    string resultJson = "{\"STATUS\": \"SUCCESS\"}";
                    retVal = SearchLineRoiResultToJsonString( result.findSuccess, roi, maskPoly, waterPoly[ 0 ], resultJson );
                    if ( GC_OK == retVal )
                    {
                        retVal = m_metaData.WriteToImageDescription( resultImgPath, resultJson );
                    }
                    else
                    {
                        resultJson = "{\"STATUS\": \"FAILURE -- Could not retrive result json string\"}";
                    }
                }
                else
                {
                    FILE_LOG( logERROR ) << "[VisApp::SaveLineFindSearchRoi] Could not write image to " << resultImgPath;
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::SaveLineFindSearchRoi] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt )
{
    GC_STATUS retVal = m_calibExec.PixelToWorld( pixelPt, worldPt );
    return retVal;
}
GC_STATUS VisApp::WorldToPixel(const cv::Point2d worldPt, cv::Point2d &pixelPt )
{
    GC_STATUS retVal = m_calibExec.WorldToPixel( worldPt, pixelPt );
    return retVal;
}

GC_STATUS VisApp::PixelToWorld( FindPointSet &ptSet )
{
    GC_STATUS retVal = m_calibExec.PixelToWorld( ptSet.ctrPixel, ptSet.ctrWorld );
    if ( GC_OK == retVal )
    {
        retVal = m_calibExec.PixelToWorld( ptSet.lftPixel, ptSet.lftWorld );
        if ( GC_OK == retVal )
        {
            retVal = m_calibExec.PixelToWorld( ptSet.rgtPixel, ptSet.rgtWorld );
        }
    }
    return retVal;
}
GC_STATUS VisApp::GetTargetSearchROI( cv::Rect &rect )
{
    GC_STATUS retVal = m_calibExec.GetTargetSearchROI( rect );
    return retVal;

}
GC_STATUS VisApp::GetCalibParams( std::string &calibParams )
{
    GC_STATUS retVal = m_calibExec.GetCalibParams( calibParams );
    return retVal;
}
GC_STATUS VisApp::GetCalibControlJson( std::string &calibJson )
{
    GC_STATUS retVal = m_calibExec.GetCalibControlJson( calibJson );
    return retVal;
}
GC_STATUS VisApp::DrawAssocPts( const cv::Mat &img, cv::Mat &overlay, std::string &err_msg )
{
    GC_STATUS retVal = m_calibExec.DrawAssocPts( img, overlay, err_msg );
    return retVal;
}
GC_STATUS VisApp::DrawCalibOverlay( const cv::Mat matIn, cv::Mat &imgMatOut )
{
    GC_STATUS retVal = m_calibExec.DrawOverlay( matIn, imgMatOut );
    return retVal;
}
GC_STATUS VisApp::DrawCalibOverlay( const cv::Mat matIn, cv::Mat &imgMatOut, const bool drawCalibScale,
                                    const bool drawCalibGrid, const bool drawSearchROI, const bool drawTargetROI )
{
    GC_STATUS retVal = m_calibExec.DrawOverlay( matIn, imgMatOut, drawCalibScale, drawCalibGrid,
                                                drawSearchROI, drawTargetROI );
    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const IMG_DISPLAY_OVERLAYS overlayTypes )
{
    GC_STATUS retVal = m_findLine.DrawResult( img, imgOut, m_findLineResult, overlayTypes );
    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const FindLineResult findLineResult,
                                       const IMG_DISPLAY_OVERLAYS overlayTypes )
{
    GC_STATUS retVal = m_findLine.DrawResult( img, imgOut, findLineResult, overlayTypes );
    return retVal;
}
GC_STATUS VisApp::FindPtSet2JsonString( const FindPointSet set, const string set_type, string &json )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        json.clear();
        stringstream ss;
        ss << "\"set_type\": \"" << set_type << "\",";
        ss << "\"anglePixel\":" << set.anglePixel << ",";
        ss << "\"angleWorld\":" << set.angleWorld << ",";
        ss << "\"lftPixel_x\":" << set.lftPixel.x << ",";
        ss << "\"lftPixel_y\":" << set.lftPixel.y << ",";
        ss << "\"lftWorld_x\":" << set.lftWorld.x << ",";
        ss << "\"lftWorld_y\":" << set.lftWorld.y << ",";
        ss << "\"ctrPixel_x\":" << set.ctrPixel.x << ",";
        ss << "\"ctrPixel_y\":" << set.ctrPixel.y << ",";
        ss << "\"ctrWorld_x\":" << set.ctrWorld.x << ",";
        ss << "\"ctrWorld_y\":" << set.ctrWorld.y << ",";
        ss << "\"rgtPixel_x\":" << set.rgtPixel.x << ",";
        ss << "\"rgtPixel_y\":" << set.rgtPixel.y << ",";
        ss << "\"rgtWorld_x\":" << set.rgtWorld.x << ",";
        ss << "\"rgtWorld_y\":" << set.rgtWorld.y;
        json = ss.str();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::FindPtSet2JsonString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::WriteFindlineResultToCSV( const std::string resultCSV, const string imgPath,
                                            const FindLineResult &result, const bool overwrite )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        bool addHeader = fs::exists( resultCSV ) ? false : true;
        ofstream csvFile;
        if ( overwrite )
            csvFile.open( resultCSV, ofstream::out );
        else
            csvFile.open( resultCSV, ofstream::out | ofstream::app );

        if ( !csvFile.is_open() )
        {
            FILE_LOG( logERROR ) << "[VisApp::WriteFindlineResultToCSV] Could not open to write" << resultCSV;
            retVal = GC_ERR;
        }
        else
        {
            if ( addHeader )
            {
                csvFile << "imgPath,";
                csvFile << "findSuccess,";

                csvFile << "timestamp,";
                csvFile << "illum_state,";
                csvFile << "waterLevel,";
                csvFile << "waterLevelAdjusted,";
                csvFile << "xRMSE, yRMSE, EuclidDistRMSE,";

                csvFile << "waterLine-octagon-angle-diff,";
                csvFile << "calcLinePts-angle,";
                csvFile << "calcLinePts-lftPixel-x,"; csvFile << "calcLinePts-lftPixel-y,";
                csvFile << "calcLinePts-ctrPixel-x,"; csvFile << "calcLinePts-ctrPixel-y,";
                csvFile << "calcLinePts-rgtPixel-x,"; csvFile << "calcLinePts-rgtPixel-y,";
                csvFile << "calcLinePts-lftWorld-x,"; csvFile << "calcLinePts-lftWorld-y,";
                csvFile << "calcLinePts-ctrWorld-x,"; csvFile << "calcLinePts-ctrWorld-y,";
                csvFile << "calcLinePts-rgtWorld-x,"; csvFile << "calcLinePts-rgtWorld-y,";

                csvFile << "octoCenter-x,"; csvFile << "octoCenter-y,";
                csvFile << "octoToSearchROIOffset-pixel," << "octoToSearchROIOffset-world,";

                csvFile << "foundPts[0]-x,"; csvFile << "foundPts[0]-y,"; csvFile << "foundPts[1]-x,"; csvFile << "foundPts[1]-y,";
                csvFile << "foundPts[2]-x,"; csvFile << "foundPts[2]-y,"; csvFile << "foundPts[3]-x,"; csvFile << "foundPts[3]-y,";
                csvFile << "foundPts[4]-x,"; csvFile << "foundPts[4]-y,"; csvFile << "foundPts[5]-x,"; csvFile << "foundPts[5]-y,";
                csvFile << "foundPts[6]-x,"; csvFile << "foundPts[6]-y,"; csvFile << "foundPts[7]-x,"; csvFile << "foundPts[7]-y,";
                csvFile << "foundPts[8]-x,"; csvFile << "foundPts[8]-y,"; csvFile << "foundPts[9]-x,"; csvFile << "foundPts[9]-y,";
                csvFile << "...";
                csvFile << endl;
            }
            csvFile << imgPath << ",";
            csvFile << ( result.findSuccess ? "true" : "false" ) << ",";
            csvFile << result.timestamp << ",";
            csvFile << result.illum_state << ",";

            csvFile << fixed << setprecision( 3 );
            csvFile << result.calcLinePts.ctrWorld.y << ",";
            csvFile << result.waterLevelAdjusted.y << ",";

            csvFile << result.calibReprojectOffset_x << ",";
            csvFile << result.calibReprojectOffset_y << ",";
            csvFile << result.calibReprojectOffset_dist << ",";

            csvFile << result.symbolToWaterLineAngle << ",";

            csvFile << result.calcLinePts.angleWorld << ",";
            csvFile << result.calcLinePts.lftPixel.x << ","; csvFile << result.calcLinePts.lftPixel.y << ",";
            csvFile << result.calcLinePts.ctrPixel.x << ","; csvFile << result.calcLinePts.ctrPixel.y << ",";
            csvFile << result.calcLinePts.rgtPixel.x << ","; csvFile << result.calcLinePts.rgtPixel.y << ",";
            csvFile << result.calcLinePts.lftWorld.x << ","; csvFile << result.calcLinePts.lftWorld.y << ",";
            csvFile << result.calcLinePts.ctrWorld.x << ","; csvFile << result.calcLinePts.ctrWorld.y << ",";
            csvFile << result.calcLinePts.rgtWorld.x << ","; csvFile << result.calcLinePts.rgtWorld.y << ",";

            csvFile << result.octoCenter.x << ","; csvFile << result.octoCenter.y << ",";
            csvFile << result.octoToSearchROIOffsetPixel << ","; csvFile << result.octoToSearchROIOffsetWorld << ",";

            for ( size_t i = 0; i < result.foundPoints.size(); ++i )
            {
                csvFile << result.foundPoints[ i ].x << ","; csvFile << result.foundPoints[ i ].y;
                if ( result.foundPoints.size() - 1 > i )
                    csvFile << ",";
            }
            csvFile << endl;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CreateCalibOverlayImage] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::EndGIF() { return m_animate.EndGIF(); }
GC_STATUS VisApp::AddImageToGIF( const cv::Mat &img ) { return m_animate.AddImageToGIF( img ); }
GC_STATUS VisApp::BeginGIF( const Size imgSize, const int imgCount, const string gifFilepath, const int delay_ms )
{
    return m_animate.BeginGIF( imgSize, imgCount, gifFilepath, delay_ms );
}

} // namespace gc
