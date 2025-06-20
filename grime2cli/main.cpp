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
#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include "arghandler.h"
#include "../algorithms/visapp.h"
#include "../algorithms/calibexecutive.h"

using namespace gc;
using namespace std;
using namespace boost;
namespace fs = std::filesystem;

// example command lines
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~ GENERAL ~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --version
// --help
// --show_metadata --source "./config/2022_demo/20220715_KOLA_GaugeCam_001.JPG"
// --make_gif --source "./config/2012_demo/06/" --result_image "/var/tmp/gaugecam/demo.gif" --scale 0.20 --delay_ms 1000

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~ STOP SIGN ~~~~~~~~~~~~~~~~~~~~~~~~~~~
// --calibrate --source "./config/2022_demo/20220715_KOLA_GaugeCam_001.JPG" --calib_json "./config/calib_stopsign.json" --result_image "/var/tmp/gaugecam/calib_result_stopsign.png"
// --find_line --timestamp_from_exif --timestamp_start_pos 0 --timestamp_format "yyyy-mm-dd-HH-MM" --source "./config/2022_demo/20220715_KOLA_GaugeCam_001.JPG" --calib_json "./config/calib.json" --csv_file "/var/tmp/gaugecam/folder_result.csv" --result_image "/var/tmp/gaugecam/find_line_result.png"
// --run_folder --timestamp_from_exif --timestamp_start_pos 0 --timestamp_format "yyyy-mm-dd-HH-MM" --source "./config/2022_demo/" --calib_json "./config/calib.json" --csv_file "/var/tmp/gaugecam/folder_result.csv" --result_folder "/var/tmp/gaugecam/" --line_roi_folder "/var/tmp/gaugecam/line_roi/"

// forward declarations
void ShowVersion();
GC_STATUS Calibrate( const Grime2CLIParams cliParams );
GC_STATUS CreateCalibrate( const Grime2CLIParams cliParams );
GC_STATUS FindWaterLevel( const Grime2CLIParams cliParams );
GC_STATUS RunFolder( const Grime2CLIParams cliParams );
GC_STATUS CreateGIF( const Grime2CLIParams cliParams );
GC_STATUS FormCalibJsonString( const Grime2CLIParams cliParams, string &json );

/** \file main.cpp
 * @brief Holds the main() function for command line use of the gaugecam libraries.
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2020, 2022, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

int main( int argc, char *argv[] )
{
    Output2FILE::Stream() = stderr;

    int ret = 0;
    if ( 2 > argc )
    {
        FILE_LOG( logERROR ) << "Not enough arguments";
        ret = -1;
    }
    else
    {
        GC_STATUS retVal = GC_OK;
        Grime2CLIParams params;
        ret = GetArgs( argc, argv, params );
        if ( 0 != ret )
        {
            cout << "{\"status\": \"FAILURE\", \"return\": " << to_string( ret ) << "}" << endl;
        }
        else
        {
            if ( CALIBRATE == params.opToPerform )
            {
                retVal = Calibrate( params );
            }
            else if ( CREATE_CALIB == params.opToPerform )
            {
                retVal = CreateCalibrate( params );
            }
            else if ( FIND_LINE == params.opToPerform )
            {
                retVal = FindWaterLevel( params );
            }
            else if ( RUN_FOLDER == params.opToPerform )
            {
                retVal = RunFolder( params );
            }
            else if ( MAKE_GIF == params.opToPerform )
            {
                retVal = CreateGIF( params );
            }
            else if ( SHOW_METADATA == params.opToPerform )
            {
                VisApp vis;
                std::string data;
                retVal = vis.GetImageData( params.src_imagePath, data );
                if ( GC_OK == retVal )
                {
                    cout << "~~~~~~~~~~~~~~~~~~~~" << endl;
                    cout << "Metadata for " << params.src_imagePath << endl;
                    cout << "~~~~~~~~~~~~~~~~~~~~" << endl;
                    cout << data << endl << endl;
                }
            }
            else if ( SHOW_VERSION == params.opToPerform )
            {
                ShowVersion();
            }
            else
            {
                if ( SHOW_HELP != params.opToPerform )
                {
                    FILE_LOG( logERROR ) << "Invalid operation specified: " << params.opToPerform << endl;
                    retVal = GC_ERR;
                }
                PrintHelp();
            }
            ret = GC_OK == retVal ? 0 : -1;
        }
    }
    return ret;
}
GC_STATUS Calibrate( const Grime2CLIParams cliParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        VisApp vis;
        cv::Mat img = cv::Mat();
        img = cv::imread( cliParams.src_imagePath );
        if ( img.empty() )
        {
            cout << "FAIL: Could not read calibration image " << cliParams.src_imagePath;
            FILE_LOG( logERROR ) << "FAIL: Could not read calibration image " << cliParams.src_imagePath;
            retVal = GC_ERR;
        }
        else
        {
            // cout << fs::canonical( cliParams.calib_jsonPath ).string() << endl;
            retVal = vis.CalibLoad( cliParams.calib_jsonPath );
            if ( GC_OK != retVal )
            {
                cout << "FAIL: Could not load calibration file " << cliParams.calib_jsonPath;
                FILE_LOG( logERROR ) << "FAIL: Could not load calibration file " << cliParams.calib_jsonPath;
            }
            else
            {
                string jsonControl;
                retVal = vis.GetCalibControlJson( jsonControl );
                if ( GC_OK == retVal )
                {
                    string err_msgs;
                    double rsmeDist, rsmeX, rsmeY;
                    retVal = vis.Calibrate( img, jsonControl, rsmeDist, rsmeX, rsmeY, err_msgs );
                    if ( !cliParams.result_imagePath.empty() )
                    {
                        cv::Mat calibOverlay;
                        retVal = vis.DrawCalibOverlay( img, calibOverlay, false, true, true, true );
                        if ( GC_OK == retVal )
                        {
                            bool bRet = imwrite( cliParams.result_imagePath, calibOverlay );
                            if ( !bRet )
                            {
                                cout << "FAIL: Could not write calibration result image " << cliParams.result_imagePath;
                                FILE_LOG( logERROR ) << "FAIL: Could not write calibration result image " << cliParams.src_imagePath;
                                retVal = GC_ERR;
                            }
                        }
                    }
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << e.what();
        retVal = GC_EXCEPT;
    }

    cout << "Calibrate: " << ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) << endl;

    return retVal;
}
GC_STATUS CreateCalibrate( const Grime2CLIParams cliParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cv::Mat img = cv::Mat();
        if ( !cliParams.src_imagePath.empty() )
        {
            img = cv::imread( cliParams.src_imagePath );
            if ( img.empty() )
            {
                cout << "FAIL: Could not read calibration image " << cliParams.src_imagePath;
                FILE_LOG( logERROR ) << "FAIL: Could not read calibration image " << cliParams.src_imagePath;
                retVal = GC_ERR;
            }
        }
        if ( 0 <= cliParams.waterline_region.lftTop.x && 0 <= cliParams.waterline_region.lftTop.y &&
             0 <= cliParams.waterline_region.lftTop.x && 0 <= cliParams.waterline_region.lftTop.x &&
             0 <= cliParams.waterline_region.lftTop.x && 0 <= cliParams.waterline_region.lftTop.x &&
             0 <= cliParams.waterline_region.lftTop.x && 0 <= cliParams.waterline_region.lftTop.x )
        {
            cv::Mat imgResult;
            string jsonStr, err_msg;
            CalibExecutive calibExec;
            CalibJsonItems items( cliParams.calib_jsonPath, ( 0 > cliParams.calib_roi.x ? false : true ),
                                  cliParams.calib_roi, cliParams.facet_length, cliParams.zero_offset,
                                  cliParams.waterline_region );
            if ( string::npos != cliParams.calib_type.find( "Octagon" ) )
            {
                retVal = calibExec.FormOctagonCalibJsonString( items, jsonStr );
            }
            if ( GC_OK == retVal )
            {
                double rmseDist, rmseX, rmseY;
                if ( cliParams.result_imagePath.empty() )
                {
                    retVal = calibExec.Calibrate( img, jsonStr, rmseDist, rmseX, rmseY, err_msg );
                }
                else
                {
                    calibExec.EnableAllOverlays();
                    retVal = calibExec.Calibrate( img, jsonStr, imgResult, rmseDist, rmseX, rmseY, err_msg, true, true );
                    bool bRet = cv::imwrite( cliParams.result_imagePath, imgResult );
                    if ( !bRet )
                    {
                        cout << "FAILURE: Could not write calibration result image " << cliParams.result_imagePath << endl;
                        FILE_LOG( logERROR ) << "FAIL: Could not write calibration result image " << cliParams.src_imagePath;
                        retVal = GC_ERR;
                    }
                }
            }
        }
        else
        {
            cout << "FAIL: Invalid waterline search region";
            FILE_LOG( logERROR ) << "FAIL: Invalid waterline search region";
            retVal = GC_ERR;
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << e.what();
        retVal = GC_EXCEPT;
    }

    cout << "Calibrate: " << ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) << endl;

    return retVal;
}
GC_STATUS RunFolder( const Grime2CLIParams cliParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !fs::is_directory( cliParams.src_imagePath ) )
        {
            FILE_LOG( logERROR ) << "Path specified is not a folder: " << cliParams.src_imagePath << endl;
            retVal = GC_ERR;
        }
        else
        {
            string ext;
            vector< string > images;
            for ( auto& p: fs::recursive_directory_iterator( cliParams.src_imagePath ) )
            {
                ext = p.path().extension().string();
                if ( ext == ".png" || ext == ".jpg" || ext == ".PNG" || ext == ".JPG" )
                {
                    images.push_back( p.path().string() );
                }
            }

            sort( images.begin(), images.end() );

            if ( images.empty() )
            {
                FILE_LOG( logERROR ) << "No images found in " << cliParams.src_imagePath << endl;
                retVal = GC_ERR;
            }
            else
            {
                string result_folder = cliParams.result_imagePath;
                if ( !result_folder.empty() )
                {
                    if ( '/' != result_folder[ result_folder.size() - 1 ] )
                        result_folder += '/';
                }

                string lineRoiFolder = cliParams.line_roi_folder;

                Grime2CLIParams cliParamsAdj = cliParams;
                for ( size_t i = 0; i < images.size(); ++i )
                {
                    if ( !result_folder.empty() )
                    {
                        cliParamsAdj.result_imagePath = result_folder +
                                fs::path( images[ i ] ).stem().string() + "_overlay.png";
                    }
                    cliParamsAdj.src_imagePath = images[ i ];
                    retVal = FindWaterLevel( cliParamsAdj );
                }
                cout << endl;
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindWaterLevel(const Grime2CLIParams cliParams )
{
    FindLineParams params;

    params.imagePath = cliParams.src_imagePath;
    params.calibFilepath = cliParams.calib_jsonPath;
    params.resultImagePath = cliParams.result_imagePath;
    params.resultCSVPath = cliParams.csvPath;
    params.timeStampFormat = cliParams.timestamp_format;
    params.timeStampType = cliParams.timestamp_type == "from_filename" ? FROM_FILENAME : FROM_EXIF;
    params.timeStampStartPos = cliParams.timestamp_startPos;
    params.lineSearchROIFolder = cliParams.line_roi_folder;

    VisApp visApp;
    string resultJson;
    FindLineResult result;
    GC_STATUS retVal = visApp.CalcLine( params, result, resultJson );
    if ( cliParams.cache_result )
    {
        ofstream cache_file( TEMP_CACHE );
        if ( cache_file.is_open() )
        {
            cache_file << resultJson << endl;
        }
    }
    cout << resultJson << endl;
    return retVal;
}
GC_STATUS CreateGIF( const Grime2CLIParams cliParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        VisApp vis;
        string ext;
        vector< std::string > images;
        for ( auto& p: fs::directory_iterator( cliParams.src_imagePath ) )
        {
            ext = p.path().extension().string();
            std::transform( ext.begin(), ext.end(), ext.begin(),
                               []( unsigned char c ){ return std::tolower( c ); } );
            if ( ext == ".png" || ext == ".jpg" || ext == ".PNG" || ext == ".JPG" )
            {
                images.push_back( p.path().string() );
            }
        }
        if ( images.empty() )
        {
            FILE_LOG( logERROR ) << "[CreateGIF] No images found in specified folder";
            retVal = GC_ERR;
        }
        else
        {
            sort( images.begin(), images.end() );

            int percent = cvRound( 100.0 / static_cast< double >( images.size() ) );
            cv::Mat img = imread( images[ 0 ], cv::IMREAD_COLOR );
            if ( img.empty() )
            {
                FILE_LOG( logERROR ) << "[CreateGIF] Could not read first image " << images[ 0 ];
                retVal = GC_ERR;
            }
            else
            {
                cout << "Initialize GIF" << endl;
                resize( img, img, cv::Size(), cliParams.scale, cliParams.scale, cv::INTER_CUBIC );
                retVal = vis.BeginGIF( img.size(), static_cast< int >( images.size() ), cliParams.result_imagePath, cliParams.delay_ms );
                if ( GC_OK == retVal )
                {
                    cout << "Add GIF frame [" << percent << "%] " << fs::path( images[ 0 ] ).filename().string() << endl;
                    retVal = vis.AddImageToGIF( img );
                    if ( GC_OK == retVal )
                    {
                        for ( size_t i = 1; i < images.size(); ++i )
                        {
                            img = imread( images[ i ], cv::IMREAD_COLOR );
                            if ( img.empty() )
                            {
                                FILE_LOG( logWARNING ) << "[CreateGIF] Could not read image " << images[ i ];
                            }
                            else
                            {
                                percent = cvRound( 100.0 * static_cast< double >( i + 1 ) / static_cast< double >( images.size() ) );
                                cout << "Add GIF frame [" << percent << "%] " << fs::path( images[ 0 ] ).filename().string() << endl;
                                resize( img, img, cv::Size(), cliParams.scale, cliParams.scale, cv::INTER_CUBIC );
                                retVal = vis.AddImageToGIF( img );
                                if ( GC_OK != retVal )
                                {
                                    FILE_LOG( logWARNING ) << "[CreateGIF] Could not add image " << images[ i ];
                                }
                            }
                        }
                    }
                    cout << "Finish GIF" << endl;
                    retVal = vis.EndGIF();
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << e.what();
        retVal = GC_EXCEPT;
    }

    cout << "Create GIF " << ( GC_OK == retVal ? "SUCCESS: " : "FAILURE: " ) << cliParams.result_imagePath << endl;

    return retVal;
}

void ShowVersion()
{
    stringstream ss;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    cout << "Application and library versions" << endl;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    cout << "  GRIME2: " << VisApp::Version()  << endl;
    cout << "  OpenCV: " << cv::getVersionString() << endl;
    cout << "ExifTool: "; fflush( stdout ); VisApp::GetExifToolVersion();
    cout << "   Boost: " << BOOST_VERSION / 100000 <<
            "." << BOOST_VERSION / 100 % 1000 <<
            "." << BOOST_VERSION % 100 << endl;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
}
