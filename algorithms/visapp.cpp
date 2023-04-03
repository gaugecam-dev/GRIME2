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
#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "timestampconvert.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = boost::filesystem;
namespace pt = property_tree;

#ifdef DEBUG_BOWTIE_FIND
#undef DEBUG_BOWTIE_FIND
#ifdef _WIN32
static const char DEBUG_FOLDER[] = "c:/gaugecam/";
#else
static const char DEBUG_FOLDER[] = "/var/tmp/water/";
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
        if ( !fs::exists( "c:/gaugecam" ) )
        {
            fs::create_directories( "c:/gaugecam" );
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
GC_STATUS VisApp::LoadCalib( const std::string calibJson, const cv::Mat &img )
{
    GC_STATUS retVal = m_calibExec.Load( calibJson, img );
    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string jsonControl, const string resultImgPath,
                             double &rmseDist, double &rmseX, double &rmseY, string &err_msg )
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
            retVal = m_calibExec.Calibrate( img, jsonControl, rmseDist, rmseX, rmseY, err_msg );
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
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string jsonControl, double &rmseDist, double &rmseX, double &rmseY, string &err_msg )
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
            retVal = m_calibExec.Calibrate( img, jsonControl, rmseDist, rmseX, rmseY, err_msg );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }
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
        FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could not retrieve exif illumination from " << filepath;
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
GC_STATUS VisApp::AdjustSearchAreaForMovement( const std::vector< LineEnds > &searchLines,
                                               std::vector< LineEnds > &searchLinesAdj, const cv::Point2d offsets )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::AdjustSearchAreaForMovement] No lines in search line vector";
            retVal = GC_ERR;
        }
        else
        {
            searchLinesAdj.clear();
            for ( size_t i = 0; i < searchLines.size(); ++i )
            {
                searchLinesAdj.push_back( LineEnds( searchLines[ i ].top + Point( cvRound( offsets.x ), cvRound( offsets.y ) ),
                                                    searchLines[ i ].bot + Point( cvRound( offsets.x ), cvRound( offsets.y ) ) ) );
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
    try
    {
        char buffer[ 256 ];
        snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
        result.msgs.push_back( buffer );

        vector< LineEnds > searchLinesAdj;
        if ( "StopSign" == m_calibExec.GetCalibType() )
        {
            result.refMovePts.setZero();
            result.foundMovePts.setZero();
            result.offsetMovePts.setZero();
            Point2d moveOffset = m_calibExec.GetStopSignMoveOffset();
#if 1
            retVal = AdjustSearchAreaForMovement( m_calibExec.SearchLines(), searchLinesAdj, moveOffset );
#else
            searchLinesAdj = m_calibExec.SearchLines();
#endif
        }
        else if ( "BowTie" == m_calibExec.GetCalibType() )
        {
            retVal = m_calibExec.MoveRefPoint( result.refMovePts.lftPixel, result.refMovePts.rgtPixel );
            if ( GC_OK == retVal )
            {
                result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                      ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
                retVal = PixelToWorld( result.refMovePts );
                if ( GC_OK != retVal )
                {
                    result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
                }
                else
                {
                    retVal = m_calibExec.FindMoveTargets( img, result.foundMovePts );
                    if ( GC_OK != retVal )
                    {
                        result.msgs.push_back( "Could not calculate move offsets" );
                    }
                    else
                    {
                        retVal = PixelToWorld( result.foundMovePts );
                        if ( GC_OK != retVal )
                        {
                            result.msgs.push_back( "Could not calculate world coordinates for found move points" );
                        }
                        else
                        {
                            result.offsetMovePts.lftPixel.x = result.foundMovePts.lftPixel.x - result.refMovePts.lftPixel.x;
                            result.offsetMovePts.lftPixel.y = result.foundMovePts.lftPixel.y - result.refMovePts.lftPixel.y;
                            result.offsetMovePts.ctrPixel.x = result.foundMovePts.ctrPixel.x - result.refMovePts.ctrPixel.x;
                            result.offsetMovePts.ctrPixel.y = result.foundMovePts.ctrPixel.y - result.refMovePts.ctrPixel.y;
                            result.offsetMovePts.rgtPixel.x = result.foundMovePts.rgtPixel.x - result.refMovePts.rgtPixel.x;
                            result.offsetMovePts.rgtPixel.y = result.foundMovePts.rgtPixel.y - result.refMovePts.rgtPixel.y;

                            result.offsetMovePts.lftWorld.x = result.foundMovePts.lftWorld.x - result.refMovePts.lftWorld.x;
                            result.offsetMovePts.lftWorld.y = result.foundMovePts.lftWorld.y - result.refMovePts.lftWorld.y;
                            result.offsetMovePts.ctrWorld.x = result.foundMovePts.ctrWorld.x - result.refMovePts.ctrWorld.x;
                            result.offsetMovePts.ctrWorld.y = result.foundMovePts.ctrWorld.y - result.refMovePts.ctrWorld.y;
                            result.offsetMovePts.rgtWorld.x = result.foundMovePts.rgtWorld.x - result.refMovePts.rgtWorld.x;
                            result.offsetMovePts.rgtWorld.y = result.foundMovePts.rgtWorld.y - result.refMovePts.rgtWorld.y;

                            result.calibOffsets.calibAngle = atan2( result.refMovePts.rgtWorld.y - result.refMovePts.lftWorld.y,
                                                                    result.refMovePts.rgtWorld.x - result.refMovePts.lftWorld.x ) * ( 180.0 / CV_PI );
                            result.calibOffsets.calibCenterPt = ( result.refMovePts.lftWorld + result.refMovePts.rgtWorld ) / 2.0;
                            result.calibOffsets.offsetAngle = atan2( result.foundMovePts.rgtWorld.y - result.foundMovePts.lftWorld.y,
                                                                     result.foundMovePts.rgtWorld.x - result.foundMovePts.lftWorld.x ) * ( 180.0 / CV_PI );
                            result.calibOffsets.offsetCenterPt = ( result.foundMovePts.lftWorld + result.foundMovePts.rgtWorld ) / 2.0;

                            retVal = AdjustSearchAreaForMovement( m_calibExec.SearchLines(), searchLinesAdj, result.offsetMovePts.ctrPixel );
                        }
                    }
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
                if ( "StopSign" == m_calibExec.GetCalibType() )
                {
                    retVal = m_calibExec.AdjustStopSignForRotation( img.size(), result.calcLinePts, result.symbolToWaterLineAngle );
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

                        sprintf( buffer, "Adjust: %.3f", result.offsetMovePts.ctrWorld.y );
                        result.msgs.push_back( buffer );

                        result.waterLevelAdjusted.x = result.calcLinePts.ctrWorld.x + result.offsetMovePts.ctrWorld.x;
                        result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y + result.offsetMovePts.ctrWorld.y;

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
//                        snprintf( buffer, 256, "Reproject error:" );
//                        result.msgs.push_back( buffer );
//                        snprintf( buffer, 256, "   x=%0.3e", result.calibReprojectOffset_x );
//                        result.msgs.push_back( buffer );
//                        snprintf( buffer, 256, "   y=%0.3e", result.calibReprojectOffset_y );
//                        result.msgs.push_back( buffer );
//                        snprintf( buffer, 256, "   Euclid. dist=%0.3e", result.calibReprojectOffset_dist );
//                        result.msgs.push_back( buffer );
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

    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params )
{
    GC_STATUS retVal = CalcLine( params, m_findLineResult );
    return retVal;
}
GC_STATUS VisApp::CalcLine( const Mat &img, const string timestamp, const bool isStopSign )
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

            if ( isStopSign )
            {
                string err_msg;
                double rmseDist, rmseX, rmseY;
                retVal = m_calibExec.Calibrate( img, "", rmseDist, rmseX, rmseY, err_msg );
                if ( GC_OK != retVal )
                {
                    FindPointSet findPtSet;
                    retVal = m_calibExec.FindMoveTargets( img, findPtSet );
                }
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
GC_STATUS VisApp::CalcLine( const FindLineParams params, FindLineResult &result, string &resultJson )
{
    GC_STATUS retVal = CalcLine( params, result );
    try
    {
        stringstream ss;
        resultJson.clear();
        if ( GC_OK != retVal )
        {
            ss << "{\"image_path\": \"FAILURE\"}";
        }
        else
        {
            ss << "{";
            ss << "\"status\": \"SUCCESS\",";
            ss << "\"image_path\": \"" << params.imagePath << "\",";
            ss << "\"calib_path\": \"" << params.calibFilepath << "\",";
            ss << "\"result_path\": \"" << params.resultImagePath << "\",";
            ss << "\"timestamp_type\": \"" << params.timeStampType << "\",";
            ss << "\"timestamp_format\": \"" << params.timeStampFormat << "\",";
            ss << "\"timestamp_start_pos\": " << params.timeStampStartPos << ",";
            ss << "\"timestamp_length\": " << params.timeStampStartPos << ",";
            ss << "\"timestamp\": \"" << result.timestamp << "\",";
            ss << "\"origCalCenter_x\": " << result.calibOffsets.calibCenterPt.x << ",";
            ss << "\"origCalCenter_y\": " << result.calibOffsets.calibCenterPt.y << ",";
            ss << "\"origCalCenter_angle\": " << result.calibOffsets.calibAngle << ",";
            ss << "\"foundCalCenter_x\": " << result.calibOffsets.offsetCenterPt.x << ",";
            ss << "\"foundCalCenter_y\": " << result.calibOffsets.offsetCenterPt.y << ",";
            ss << "\"foundCalCenter_angle\": " << result.calibOffsets.offsetAngle << ",";

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
            else if ( FROM_EXTERNAL == params.timeStampType )
            {
                FILE_LOG( logERROR ) << "Timestamp passed into method not yet implemented";
                retVal = GC_ERR;
            }

            if ( GC_OK != retVal )
            {
                result.msgs.push_back( "Timestamp failure. Check source, format, and start position of timestamp" );
            }
            else
            {
                bool isOK = true;
                if ( ( params.imagePath != params.resultImagePath ) && !params.resultImagePath.empty() )
                {
                    isOK = imwrite( params.resultImagePath, img );
                    if ( !isOK )
                    {
                        result.msgs.push_back( "Could not save result image to " + params.resultImagePath );
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not save result image to " << params.resultImagePath;
                        retVal = GC_ERR;
                    }
                }
                if ( GC_OK == retVal )
                {
                    if ( params.isStopSignCalib || ( params.calibFilepath != m_calibFilepath && !params.isStopSignCalib ) )
                    {
                        Mat noImg = Mat();
                        retVal = m_calibExec.Load( params.calibFilepath, params.isStopSignCalib ? img : noImg );
                        if ( GC_OK != retVal )
                        {
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
                    if ( GC_OK != retVal )
                    {
                        result.findSuccess = false;
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                        retVal = GC_ERR;
                    }
                    m_findLineResult = result;
                    if ( !params.resultCSVPath.empty() )
                    {
                        retVal = WriteFindlineResultToCSV( params.resultCSVPath, params.imagePath, result );
                    }
                    if ( !params.resultImagePath.empty() )
                    {
                        Mat color;
                        retVal = DrawLineFindOverlay( img, color, result );
                        if ( GC_OK == retVal )
                        {
                            bool isOk = imwrite( params.resultImagePath, color );
                            if ( !isOk)
                            {
                                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not write result image to " << params.resultImagePath;
                                retVal = GC_ERR;
                            }
                        }
                    }
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
                                    const bool drawCalibGrid, const bool drawMoveROIs,
                                    const bool drawSearchROI, const bool drawTargetROI )
{
    GC_STATUS retVal = m_calibExec.DrawOverlay( matIn, imgMatOut, drawCalibScale, drawCalibGrid,
                                                drawMoveROIs, drawSearchROI, drawTargetROI, m_findLineResult.offsetMovePts.ctrPixel );
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

                csvFile << "waterLine-stopSign-angle-diff,";
                csvFile << "calcLinePts-angle,";
                csvFile << "calcLinePts-lftPixel-x,"; csvFile << "calcLinePts-lftPixel-y,";
                csvFile << "calcLinePts-ctrPixel-x,"; csvFile << "calcLinePts-ctrPixel-y,";
                csvFile << "calcLinePts-rgtPixel-x,"; csvFile << "calcLinePts-rgtPixel-y,";
                csvFile << "calcLinePts-lftWorld-x,"; csvFile << "calcLinePts-lftWorld-y,";
                csvFile << "calcLinePts-ctrWorld-x,"; csvFile << "calcLinePts-ctrWorld-y,";
                csvFile << "calcLinePts-rgtWorld-x,"; csvFile << "calcLinePts-rgtWorld-y,";

                csvFile << "refMovePts-angle,";
                csvFile << "refMovePts-lftPixel-x,"; csvFile << "refMovePts-lftPixel-y,";
                csvFile << "refMovePts-ctrPixel-x,"; csvFile << "refMovePts-ctrPixel-y,";
                csvFile << "refMovePts-rgtPixel-x,"; csvFile << "refMovePts-rgtPixel-y,";
                csvFile << "refMovePts-lftWorld-x,"; csvFile << "refMovePts-lftWorld-y,";
                csvFile << "refMovePts-ctrWorld-x,"; csvFile << "refMovePts-ctrWorld-y,";
                csvFile << "refMovePts-rgtWorld-x,"; csvFile << "refMovePts-rgtWorld-y,";

                csvFile << "foundMovePts-angle,";
                csvFile << "foundMovePts-lftPixel-x,"; csvFile << "foundMovePts-lftPixel-y,";
                csvFile << "foundMovePts-ctrPixel-x,"; csvFile << "foundMovePts-ctrPixel-y,";
                csvFile << "foundMovePts-rgtPixel-x,"; csvFile << "foundMovePts-rgtPixel-y,";
                csvFile << "foundMovePts-lftWorld-x,"; csvFile << "foundMovePts-lftWorld-y,";
                csvFile << "foundMovePts-ctrWorld-x,"; csvFile << "foundMovePts-ctrWorld-y,";
                csvFile << "foundMovePts-rgtWorld-x,"; csvFile << "foundMovePts-rgtWorld-y,";

                csvFile << "offsetMovePts-angle,";
                csvFile << "offsetMovePts-lftPixel-x,"; csvFile << "offsetMovePts-lftPixel-y,";
                csvFile << "offsetMovePts-ctrPixel-x,"; csvFile << "offsetMovePts-ctrPixel-y,";
                csvFile << "offsetMovePts-rgtPixel-x,"; csvFile << "offsetMovePts-rgtPixel-y,";
                csvFile << "offsetMovePts-lftWorld-x,"; csvFile << "offsetMovePts-lftWorld-y,";
                csvFile << "offsetMovePts-ctrWorld-x,"; csvFile << "offsetMovePts-ctrWorld-y,";
                csvFile << "offsetMovePts-rgtWorld-x,"; csvFile << "offsetMovePts-rgtWorld-y,";

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

            csvFile << result.refMovePts.angleWorld << ",";
            csvFile << result.refMovePts.lftPixel.x << ","; csvFile << result.refMovePts.lftPixel.y << ",";
            csvFile << result.refMovePts.ctrPixel.x << ","; csvFile << result.refMovePts.ctrPixel.y << ",";
            csvFile << result.refMovePts.rgtPixel.x << ","; csvFile << result.refMovePts.rgtPixel.y << ",";
            csvFile << result.refMovePts.lftWorld.x << ","; csvFile << result.refMovePts.lftWorld.y << ",";
            csvFile << result.refMovePts.ctrWorld.x << ","; csvFile << result.refMovePts.ctrWorld.y << ",";
            csvFile << result.refMovePts.rgtWorld.x << ","; csvFile << result.refMovePts.rgtWorld.y << ",";

            csvFile << result.foundMovePts.angleWorld << ",";
            csvFile << result.foundMovePts.lftPixel.x << ","; csvFile << result.foundMovePts.lftPixel.y << ",";
            csvFile << result.foundMovePts.ctrPixel.x << ","; csvFile << result.foundMovePts.ctrPixel.y << ",";
            csvFile << result.foundMovePts.rgtPixel.x << ","; csvFile << result.foundMovePts.rgtPixel.y << ",";
            csvFile << result.foundMovePts.lftWorld.x << ","; csvFile << result.foundMovePts.lftWorld.y << ",";
            csvFile << result.foundMovePts.ctrWorld.x << ","; csvFile << result.foundMovePts.ctrWorld.y << ",";
            csvFile << result.foundMovePts.rgtWorld.x << ","; csvFile << result.foundMovePts.rgtWorld.y << ",";

            csvFile << result.offsetMovePts.angleWorld << ",";
            csvFile << result.offsetMovePts.lftPixel.x << ","; csvFile << result.offsetMovePts.lftPixel.y << ",";
            csvFile << result.offsetMovePts.ctrPixel.x << ","; csvFile << result.offsetMovePts.ctrPixel.y << ",";
            csvFile << result.offsetMovePts.rgtPixel.x << ","; csvFile << result.offsetMovePts.rgtPixel.y << ",";
            csvFile << result.offsetMovePts.lftWorld.x << ","; csvFile << result.offsetMovePts.lftWorld.y << ",";
            csvFile << result.offsetMovePts.ctrWorld.x << ","; csvFile << result.offsetMovePts.ctrWorld.y << ",";
            csvFile << result.offsetMovePts.rgtWorld.x << ","; csvFile << result.offsetMovePts.rgtWorld.y << ",";

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
