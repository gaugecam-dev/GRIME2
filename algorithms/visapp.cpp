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
#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "animate.h"
#include "timestampconvert.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;
namespace pt = property_tree;

#ifdef DEBUG_BOWTIE_FIND
#undef DEBUG_BOWTIE_FIND
#ifdef _WIN32
    static const string DEBUG_FOLDER = "c:/gaugecam/";
#else
    static const string DEBUG_FOLDER = "/var/tmp/gaugecam/";
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
GC_STATUS VisApp::LoadCalib( const std::string calibJson )
{
    GC_STATUS retVal = m_calibExec.Load( calibJson );
    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string jsonControl, Mat &imgOut )
{
    GC_STATUS retVal = m_calibExec.Calibrate( imgFilepath, jsonControl, imgOut );
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
            FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could  not retrieve exif image data from " << filepath;
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
FindLineResult VisApp::GetFindLineResult()
{
    return m_findLineResult;
}
void VisApp::SetFindLineResult( const FindLineResult result )
{
    m_findLineResult = result;
}
GC_STATUS VisApp::AdjustSearchAreaForMovement( const std::vector< LineEnds > &searchLines,
                                               std::vector< LineEnds > &searchLinesAdj, const cv::Point offsets )
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
                searchLinesAdj.push_back( LineEnds( searchLines[ i ].top + offsets, searchLines[ i ].bot + offsets ) );
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
        retVal = m_calibExec.MoveRefPoint( result.refMovePts.lftPixel, result.refMovePts.rgtPixel );
        if ( GC_OK == retVal )
        {
            FindPointSet offsetPts;
            result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                  ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
            retVal = PixelToWorld( result.refMovePts );
            if ( GC_OK != retVal )
            {
                result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
            }
            else
            {
                char buffer[ 256 ];
                snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
                result.msgs.push_back( buffer );

                result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );

                snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                result.msgs.push_back( buffer );

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

                        sprintf( buffer, "Adjust: %.3f", result.offsetMovePts.ctrWorld.y );
                        result.msgs.push_back( buffer );



                        vector< LineEnds > searchLinesAdj;
                        retVal = AdjustSearchAreaForMovement( m_calibExec.SearchLines(), searchLinesAdj, result.offsetMovePts.ctrWorld );
                        if ( GC_OK == retVal )
                        {
                            retVal = m_findLine.Find( img, searchLinesAdj, m_calibExec.TargetRoi(), result );
                            if ( GC_OK != retVal )
                            {
                                m_findLineResult = result;
                                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                                retVal = GC_ERR;
                            }
                            else
                            {
                                retVal = PixelToWorld( result.calcLinePts );
                                if ( GC_OK != retVal )
                                {
                                    result.msgs.push_back( "Could not calculate world coordinates for found line points" );
                                }
                                else
                                {
                                    result.calcLinePts.angleWorld = atan( ( result.offsetMovePts.rgtWorld.y - result.offsetMovePts.lftWorld.y ) /
                                                                          ( result.offsetMovePts.rgtWorld.x - result.offsetMovePts.lftWorld.x ) );
                                    snprintf( buffer, 256, "Angle: %.3f", result.calcLinePts.angleWorld );
                                    result.msgs.push_back( buffer );

                                    result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y;
                                    result.calcLinePts.ctrWorld.y -= result.offsetMovePts.ctrWorld.y;
                                    snprintf( buffer, 256, "Level (adj): %.3f", result.waterLevelAdjusted.y );
                                    result.msgs.push_back( buffer );
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
GC_STATUS VisApp::CalcLine( const Mat &img, const string timestamp )
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
#if 1
            char buffer[ 256 ];
            result.timestamp = timestamp;
            snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
            result.msgs.push_back( buffer );

            retVal = CalcFindLine( img, result );
            if ( GC_OK != retVal )
            {
                result.findSuccess = false;
                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                retVal = GC_ERR;
            }
#else
            retVal = m_findLine.Find( img, m_calibExec.SearchLines(), m_calibExec.TargetRoi(), result );
            if ( GC_OK != retVal )
            {
                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                retVal = GC_ERR;
            }
            else
            {
                char buffer[ 256 ];
                result.timestamp = timestamp;
                snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
                result.msgs.push_back( buffer );

                result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );

                retVal = PixelToWorld( result.calcLinePts );
                if ( GC_OK != retVal )
                {
                    result.msgs.push_back( "Could not calculate world coordinates for found line points" );
                }
                else
                {
                    retVal = m_calibExec.MoveRefPoint( result.refMovePts.lftPixel, result.refMovePts.rgtPixel );
                    if ( GC_OK == retVal )
                    {
                        result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                              ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
                        FindPointSet offsetPts;
                        retVal = PixelToWorld( result.refMovePts );
                        if ( GC_OK != retVal )
                        {
                            result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
                        }
                        else
                        {
                            snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                            result.msgs.push_back( buffer );
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

                                    snprintf( buffer, 256, "Adjust: %.3f", result.offsetMovePts.ctrWorld.y );
                                    result.msgs.push_back( buffer );
                                    result.waterLevelAdjusted.x = result.calcLinePts.ctrWorld.x - result.offsetMovePts.ctrWorld.x;
                                    result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y - result.offsetMovePts.ctrWorld.y;

                                    result.calcLinePts.angleWorld = atan( ( result.offsetMovePts.rgtWorld.y - result.offsetMovePts.lftWorld.y ) /
                                                                          ( result.offsetMovePts.rgtWorld.x - result.offsetMovePts.lftWorld.x ) );
                                    snprintf( buffer, 256, "Angle: %.3f", result.calcLinePts.angleWorld );
                                    result.msgs.push_back( buffer );

                                    snprintf( buffer, 256, "Level (adj): %.3f", result.waterLevelAdjusted.y );
                                    result.msgs.push_back( buffer );
                                }
                            }
                        }
                    }
                }
            }
#endif
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
    if ( GC_OK == retVal )
    {
        try
        {
            resultJson.clear();
            stringstream ss;
            ss << "{";
            ss << "\"image_path\": \"" << params.imagePath << "\",";
            ss << "\"calib_path\": \"" << params.calibFilepath << "\",";
            ss << "\"result_path\": \"" << params.resultImagePath << "\",";
            ss << "\"timestamp_type\": \"" << params.timeStampType << "\",";
            ss << "\"timestamp_format\": \"" << params.timeStampFormat << "\",";
            ss << "\"timestamp_start_pos\": " << params.timeStampStartPos << ",";
            ss << "\"timestamp_length\": " << params.timeStampStartPos << ",";
            ss << "\"status\":  \"" << ( result.findSuccess ? "SUCCESS" : "FAIL" ) << "\",";
            ss << "\"timestamp\": \"" << result.timestamp << "\",";
            ss << "\"waterLevelAdjusted_x\": " << result.waterLevelAdjusted.x << ",";
            ss << "\"waterLevelAdjusted_y\": " << result.waterLevelAdjusted.y << ",";

            string json;
            retVal = FindPtSet2JsonString( result.calcLinePts, "calc_line_pts", json );
            if ( GC_OK == retVal )
            {
                ss << json << ",";
                retVal = FindPtSet2JsonString( result.refMovePts, "ref_move_pts", json );
                if ( GC_OK == retVal )
                {
                    ss << json << ",";
                    retVal = FindPtSet2JsonString( result.foundMovePts, "found_move_pts", json );
                    if ( GC_OK == retVal )
                    {
                        ss << json << ",";
                        retVal = FindPtSet2JsonString( result.offsetMovePts, "offset_move_pts", json );
                        if ( GC_OK == retVal )
                        {
                            ss << json << ", \"found_pts\": [";
                            for ( size_t i = 0; i < result.foundPoints.size(); ++i )
                            {
                                ss << "{\"x\": " << result.foundPoints[ i ].x << ",";
                                ss <<  "\"y\": " << result.foundPoints[ i ].y << "}";
                                if ( result.foundPoints.size() - 1 != i )
                                    ss << ",";
                            }
                            ss << "],";
                            ss << json << ", \"messages\": [";
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
                }
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLine] " << e.what();
            FILE_LOG( logERROR ) << "Image=" << params.imagePath << " calib=" << params.calibFilepath;
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        result.clear();
        cv::Mat img = imread( params.imagePath, IMREAD_GRAYSCALE );
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
            if ( GC_OK == retVal )
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
                if ( params.calibFilepath != m_calibFilepath && GC_OK == retVal )
                {
                    retVal = m_calibExec.Load( params.calibFilepath );
                    if ( GC_OK != retVal )
                    {
                        result.msgs.push_back( "Could not load calibration" );
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not load calibration=" << params.calibFilepath ;
                        retVal = GC_ERR;
                    }
                }
                if ( GC_OK == retVal)
                {
                    m_calibFilepath = params.calibFilepath;

#if 1
                    char buffer[ 256 ];
                    snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
                    result.msgs.push_back( buffer );

                    retVal = CalcFindLine( img, result );
                    if ( GC_OK != retVal )
                    {
                        result.findSuccess = false;
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                        retVal = GC_ERR;
                    }
#else
                    retVal = m_findLine.Find( img, m_calibExec.SearchLines(), m_calibExec.TargetRoi(), result );
                    if ( GC_OK != retVal )
                    {
                        m_findLineResult = result;
                        FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image=" << params.imagePath << " calib=" << params.calibFilepath;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        retVal = PixelToWorld( result.calcLinePts );
                        if ( GC_OK != retVal )
                        {
                            result.msgs.push_back( "Could not calculate world coordinates for found line points" );
                        }
                        else
                        {
                            retVal = m_calibExec.MoveRefPoint( result.refMovePts.lftPixel, result.refMovePts.rgtPixel );
                            if ( GC_OK == retVal )
                            {
                                FindPointSet offsetPts;
                                result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                                      ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
                                retVal = PixelToWorld( result.refMovePts );
                                if ( GC_OK != retVal )
                                {
                                    result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
                                }
                                else
                                {
                                    char buffer[ 256 ];
                                    snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
                                    result.msgs.push_back( buffer );

                                    result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );

                                    snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                                    result.msgs.push_back( buffer );

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

                                            sprintf( buffer, "Adjust: %.3f", result.offsetMovePts.ctrWorld.y );
                                            result.msgs.push_back( buffer );

                                            result.calcLinePts.angleWorld = atan( ( result.offsetMovePts.rgtWorld.y - result.offsetMovePts.lftWorld.y ) /
                                                                                  ( result.offsetMovePts.rgtWorld.x - result.offsetMovePts.lftWorld.x ) );
                                            snprintf( buffer, 256, "Angle: %.3f", result.calcLinePts.angleWorld );
                                            result.msgs.push_back( buffer );

                                            result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y - result.offsetMovePts.ctrWorld.y;
                                            snprintf( buffer, 256, "Level (adj): %.3f", result.waterLevelAdjusted.y );
                                            result.msgs.push_back( buffer );
                                        }
                                    }
                                }
                            }
                        }
                    }
#endif
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
GC_STATUS VisApp::DrawCalibOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                                    const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = m_calibExec.DrawOverlay( matIn, imgMatOut, drawCalib, drawMoveROIs, drawSearchROI );
    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const LineDrawType overlayTypes )
{
    GC_STATUS retVal = m_findLine.DrawResult( img, imgOut, m_findLineResult, overlayTypes );
    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const FindLineResult findLineResult,
                                       const LineDrawType overlayTypes )
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
                csvFile << "waterLevel,";
                csvFile << "waterLevelAdjusted,";

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

            csvFile << fixed << setprecision( 3 );
            csvFile << result.calcLinePts.ctrWorld.y << ",";
            csvFile << result.waterLevelAdjusted.y << ",";

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
GC_STATUS VisApp::CreateAnimation( const std::string imageFolder, const std::string animationFilepath, const double fps, const double scale )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !fs::is_directory( imageFolder ) )
        {
            FILE_LOG( logERROR ) << "[VisApp::CreateAnimation] Path specified is not a folder: " << imageFolder << endl;
            retVal = GC_ERR;
        }
        else
        {
            string ext;
            vector< string > images;
            for ( auto& p: fs::recursive_directory_iterator( imageFolder ) )
            {
                ext = p.path().extension().string();
                if ( ext == ".png" || ext == ".jpg" ||
                     ext == ".PNG" || ext == ".JPG" )
                {
                    images.push_back( p.path().string() );
                }
            }

            if ( images.empty() )
            {
                FILE_LOG( logERROR ) << "No images found in " << imageFolder << endl;
                retVal = GC_ERR;
            }
            else
            {
                sort( images.begin(), images.end() );

                Animate animate;
                retVal = animate.CreateCacheFolder();
                if ( GC_OK == retVal )
                {
                    Mat img;
                    char buffer[ 512 ];
                    for ( size_t i = 0; i < static_cast< size_t >( std::min( 1000, static_cast< int >( images.size() ) ) ); ++i )
                    {
                        img = imread( images[ i ], IMREAD_ANYCOLOR );
                        if ( img.empty() )
                        {
                            FILE_LOG( logWARNING ) << "Could not read frame: " << images[ i ] << " for animation";
                        }
                        else
                        {
                            sprintf( buffer, "image%03d.png", static_cast< int >( i ) );
                            retVal = animate.AddFrame( string( buffer ), img );
                        }
                    }
                    retVal = animate.Create( animationFilepath, fps, scale );
                    retVal = animate.RemoveCacheFolder();
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CreateAnimation] " <<diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
