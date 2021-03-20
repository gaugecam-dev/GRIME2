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
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "timestampconvert.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

static const double MIN_BOWTIE_FIND_SCORE = 0.55;

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
    GC_STATUS retVal = m_findLine.InitBowtieSearch( GC_BOWTIE_TEMPLATE_DIM, Size( GC_IMAGE_SIZE_WIDTH, GC_IMAGE_SIZE_HEIGHT ) );
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[VisApp::VisApp] Could not initialize bowtie templates for move detection";
    }
    else
    {
        GC_STATUS retVal = m_findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, Size( GC_IMAGE_SIZE_WIDTH, GC_IMAGE_SIZE_HEIGHT ) );
        if ( GC_OK != retVal )
        {
            FILE_LOG( logERROR ) << "[VisApp::VisApp] Could not initialize bowtie templates for calibration";
        }
    }
}
GC_STATUS VisApp::LoadCalib( const std::string calibJson )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = m_calib.Load( calibJson );
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::LoadCalib] " << e.what();
        FILE_LOG( logERROR ) << "Calib file=" << calibJson ;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string worldCoordsCsv, const string calibJson, const string resultImagepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat matOut;
        retVal = Calibrate( imgFilepath, worldCoordsCsv, calibJson, matOut, !resultImagepath.empty() );
        if ( GC_OK == retVal && !resultImagepath.empty() )
        {
            bool bRet = imwrite( resultImagepath, matOut );
            if ( !bRet )
            {
                FILE_LOG( logERROR ) << "Could not write image overlay to file " << resultImagepath;
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " world coords csv=" << worldCoordsCsv;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::Calibrate( const string imgFilepath, const string worldCoordsCsv, const string calibJson, Mat &imgOut, const bool createOverlay )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat img = imread( imgFilepath, IMREAD_GRAYSCALE );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::Calibrate] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            vector< vector< Point2d > > worldCoords;
            retVal = ReadWorldCoordsFromCSV( worldCoordsCsv, worldCoords );
            if ( GC_OK == retVal )
            {
#ifdef DEBUG_BOWTIE_FIND
                retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE, DEBUG_FOLDER + "bowtie_find.png" );
#else
                retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE );
#endif
                if ( GC_OK == retVal )
                {
                    vector< vector< Point2d > > pixelCoords;
                    retVal = m_findCalibGrid.GetFoundPoints( pixelCoords );
                    if ( GC_OK == retVal )
                    {
                        if ( pixelCoords.size() != worldCoords.size() )
                        {
                            FILE_LOG( logERROR ) << "[VisApp::Calibrate] Found pixel array row count does not equal world array count";
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
                                    FILE_LOG( logERROR ) << "[VisApp::Calibrate] Found pixel array column count does not equal world array count";
                                    retVal = GC_ERR;
                                    break;
                                }
                                for ( size_t j = 0; j < pixelCoords[ i ].size(); ++j )
                                {
                                    pixPtArray.push_back( pixelCoords[ i ][ j ] );
                                    worldPtArray.push_back( worldCoords[ i ][ j ] );
                                }
                            }
                            retVal = m_calib.Calibrate( pixPtArray, worldPtArray, Size( 2, 4 ), img.size(), img, imgOut, createOverlay );
                            if ( GC_OK == retVal )
                            {
                                retVal = m_calib.Save( calibJson );
                            }
                        }
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::Calibrate] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " world coords csv=" << worldCoordsCsv;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::GetImageTimestamp( const std::string filepath, std::string &timestamp )
{
    GC_STATUS retVal = m_metaData.GetExifData( filepath, "DateTimeOriginal", timestamp );
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[VisApp::ListMetadata] Could  not retrieve exif image data from " << filepath;
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
GC_STATUS VisApp::CalcLine( const FindLineParams params, const string timestamp )
{
    GC_STATUS retVal = CalcLine( params, timestamp, m_findLineResult );
    return retVal;
}
GC_STATUS VisApp::CalcLine( const Mat &img, const string timestamp )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        FindLineResult result;
        result.timestamp = timestamp;
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLine] Empty image";
            retVal = GC_ERR;
        }
        else
        {
            retVal = m_findLine.Find( img, m_calib.SearchLines(), result );
            if ( GC_OK != retVal )
            {
                FILE_LOG( logERROR ) << "[VisApp::CalcLine] Could not calc line in image";
                retVal = GC_ERR;
            }
            else
            {
                result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );
                char buffer[ 256 ];
                snprintf( buffer, 256, "Timestamp: %s", result.timestamp.c_str() );
                result.msgs.push_back( buffer );

                retVal = PixelToWorld( result.calcLinePts );
                if ( GC_OK != retVal )
                {
                    result.msgs.push_back( "Could not calculate world coordinates for found line points" );
                }
                else
                {
                    retVal = m_findLine.SetMoveTargetROI( img, m_calib.MoveSearchROI( true ), true );
                    if ( GC_OK != retVal )
                    {
                        result.msgs.push_back( "Could not set left move target search region" );
                    }
                    else
                    {
                        retVal = m_findLine.SetMoveTargetROI( img, m_calib.MoveSearchROI( false ), false );
                        if ( GC_OK != retVal )
                        {
                            result.msgs.push_back( "Could not set right move target search region" );
                        }
                        else
                        {
                            FindPointSet offsetPts;
                            result.refMovePts.lftPixel = m_calib.MoveRefPoint( true );
                            result.refMovePts.rgtPixel = m_calib.MoveRefPoint( false );
                            result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                                  ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
                            retVal = PixelToWorld( result.refMovePts );
                            if ( GC_OK != retVal )
                            {
                                result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
                            }
                            else
                            {
                                snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                                result.msgs.push_back( buffer );
                                retVal = m_findLine.FindMoveTargets( img, result.foundMovePts );
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
                                        result.waterLevelAdjusted.x = result.calcLinePts.ctrWorld.x - result.offsetMovePts.ctrWorld.x;
                                        result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y - result.offsetMovePts.ctrWorld.y;
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
        m_findLineResult = result;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLine] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::CalcLine( const FindLineParams params, const string timestamp, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        result.clear();
        result.timestamp = timestamp;
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
                                                                     params.timeStampStartPos, params.timeStampLength,
                                                                     params.timeStampFormat, result.timestamp );
            }
            else if ( FROM_EXIF == params.timeStampType )
            {
                string timestampTemp;
                retVal = GetImageTimestamp( params.imagePath, timestampTemp );
                if ( GC_OK == retVal )
                {
                    retVal = GcTimestampConvert::GetTimestampFromString( timestampTemp,
                                                                         params.timeStampStartPos, params.timeStampLength,
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
                    retVal = m_calib.Load( params.calibFilepath );
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

                    retVal = m_findLine.Find( img, m_calib.SearchLines(), result );
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
                            retVal = m_findLine.SetMoveTargetROI( img, m_calib.MoveSearchROI( true ), true );
                            if ( GC_OK != retVal )
                            {
                                result.msgs.push_back( "Could not set left move target search region" );
                            }
                            else
                            {
                                retVal = m_findLine.SetMoveTargetROI( img, m_calib.MoveSearchROI( false ), false );
                                if ( GC_OK != retVal )
                                {
                                    result.msgs.push_back( "Could not set right move target search region" );
                                }
                                else
                                {
                                    FindPointSet offsetPts;
                                    result.refMovePts.lftPixel = m_calib.MoveRefPoint( true );
                                    result.refMovePts.rgtPixel = m_calib.MoveRefPoint( false );
                                    result.refMovePts.ctrPixel = Point2d( ( result.refMovePts.lftPixel.x + result.refMovePts.rgtPixel.x ) / 2.0,
                                                                          ( result.refMovePts.lftPixel.y + result.refMovePts.rgtPixel.y ) / 2.0 );
                                    retVal = PixelToWorld( result.refMovePts );
                                    if ( GC_OK != retVal )
                                    {
                                        result.msgs.push_back( "Could not calculate world coordinates for move reference points" );
                                    }
                                    else
                                    {
                                        result.msgs.push_back( "FindStatus: " + string( GC_OK == retVal ? "SUCCESS" : "FAIL" ) );
                                        char buffer[ 256 ];
                                        snprintf( buffer, 256, "Level: %.3f", result.calcLinePts.ctrWorld.y );
                                        result.msgs.push_back( buffer );
                                        retVal = m_findLine.FindMoveTargets( img, result.foundMovePts );
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
                                                result.waterLevelAdjusted.y = result.calcLinePts.ctrWorld.y - result.offsetMovePts.ctrWorld.y;
                                                snprintf( buffer, 256, "Level (adj): %.3f", result.waterLevelAdjusted.y );
                                                result.msgs.push_back( buffer );
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    m_findLineResult = result;
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
GC_STATUS VisApp::WorldToPixel( const Point2d worldPt, Point2d &pixelPt )
{
    GC_STATUS retVal = m_calib.WorldToPixel( worldPt, pixelPt );
    return retVal;
}
GC_STATUS VisApp::PixelToWorld( FindPointSet &ptSet )
{
    GC_STATUS retVal = m_calib.PixelToWorld( ptSet.ctrPixel, ptSet.ctrWorld );
    if ( GC_OK == retVal )
    {
        retVal = m_calib.PixelToWorld( ptSet.lftPixel, ptSet.lftWorld );
        if ( GC_OK == retVal )
        {
            retVal = m_calib.PixelToWorld( ptSet.rgtPixel, ptSet.rgtWorld );
        }
    }
    return retVal;
}
GC_STATUS VisApp::ReadWorldCoordsFromCSV( const string csvFilepath, vector< vector< Point2d > > &worldCoords )
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
        FILE_LOG( logERROR ) << "[VisApp::ReadWorldCoordsFromCSV] " << diagnostic_information( e );
        FILE_LOG( logERROR ) << "Could not read CSV filepath=" << csvFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::DrawCalibOverlay( const cv::Mat matIn, cv::Mat &imgMatOut )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        CalibModel model = m_calib.GetModel();
        retVal = m_calib.Calibrate( model.pixelPoints, model.worldPoints,
                                    model.gridSize, matIn.size(), matIn, imgMatOut, true );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CreateCalibOverlayImage] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut )
{
    GC_STATUS retVal = m_findLine.DrawResult( img, imgOut, m_findLineResult );
    return retVal;
}
GC_STATUS VisApp::DrawLineFindOverlay( const cv::Mat &img, cv::Mat &imgOut, const FindLineResult findLineResult )
{
    GC_STATUS retVal = m_findLine.DrawResult( img, imgOut, findLineResult );
    return retVal;
}
GC_STATUS VisApp::WriteFindlineResultToCSV( const std::string resultCSV, const string imgResultPath,
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
                csvFile << "imgResultPath,";
                csvFile << "findSuccess,";

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
            csvFile << imgResultPath << ",";
            csvFile << ( result.findSuccess ? "true" : "false" ) << ",";

            csvFile << fixed << setprecision( 3 );
            csvFile << result.calcLinePts.ctrWorld.y << ",";
            csvFile << result.waterLevelAdjusted << ",";

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

} // namespace gc
