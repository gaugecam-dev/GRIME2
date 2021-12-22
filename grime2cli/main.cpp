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
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "arghandler.h"
#include "../algorithms/visapp.h"

using namespace std;
using namespace gc;

// example command lines
// --version
// --show_help
// --calibrate "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/2012_demo/06/NRmarshDN-12-06-30-10-30.jpg" --target_roi 280 20 500 580 --calib_json "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/calib.json" --csv_file "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/calibration_target_world_coordinates.csv" --result_image "/var/tmp/water/calib_result.png"
// --show_metadata "/home/kchapman/data/idaho_power/bad_cal_bad_line_find/TREK0003.jpg"
// --find_line --timestamp_from_filename --timestamp_start_pos 10 --timestamp_format "yy-mm-dd-HH-MM" "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/2012_demo/06/NRmarshDN-12-06-30-10-30.jpg" --calib_json "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/calib.json" --result_image "/var/tmp/water/find_line_result.png"
// --run_folder --timestamp_from_filename --timestamp_start_pos 10 --timestamp_format "yy-mm-dd-HH-MM" "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/2012_demo/06/" --calib_json "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop_Qt_5_15_2_GCC_64bit-Debug/config/calib.json" --csv_file "/var/tmp/water/folder.csv" --result_folder "/var/tmp/water/"
// --make_gif "/media/kchapman/Elements/Projects/GRIME2/gcgui/config/2012_demo/06/" --result_image "/var/tmp/water/demo.gif" --scale 0.20 --delay_ms 1000

// forward declarations
void ShowVersion();
GC_STATUS Calibrate( const Grime2CLIParams cliParams );
GC_STATUS FindWaterLevel( const Grime2CLIParams cliParams );
GC_STATUS RunFolder( const Grime2CLIParams cliParams );
GC_STATUS CreateGIF( const Grime2CLIParams cliParams );
GC_STATUS FormCalibJsonString( const Grime2CLIParams cliParams, string &json );

/** \file main.cpp
 * @brief Holds the main() function for command line use of the h2o_cli libraries.
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2020, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
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
        if ( 0 == ret )
        {
            if ( CALIBRATE == params.opToPerform )
            {
                retVal = Calibrate( params );
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
        string jsonString;
        retVal = FormCalibJsonString( cliParams, jsonString );
        if ( GC_OK == retVal )
        {
            retVal = vis.Calibrate( cliParams.src_imagePath, jsonString );
            if ( GC_OK == retVal && !cliParams.result_imagePath.empty() )
            {
                cv::Mat imgOut;
                bool bRet = cv::imwrite( cliParams.result_imagePath, imgOut );
                if ( !bRet )
                {
                    FILE_LOG( logWARNING ) << "Could not save calibration result image " << cliParams.result_imagePath;
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
                if ( ext == ".png" || ext == ".jpg" )
                {
                    images.push_back( p.path().string() );
                }
            }

            if ( images.empty() )
            {
                FILE_LOG( logERROR ) << "No images found in " << cliParams.src_imagePath << endl;
                retVal = GC_ERR;
            }
            else
            {
                FindLineParams params;

                params.calibFilepath = cliParams.calib_jsonPath;
                params.resultCSVPath = cliParams.csvPath;
                string result_folder = cliParams.result_imagePath;
                if ( !result_folder.empty() )
                {
                    if ( '/' != result_folder[ result_folder.size() - 1 ] )
                        result_folder += '/';
                }
                params.timeStampFormat = cliParams.timestamp_format;
                params.timeStampType = cliParams.timestamp_type == "from_filename" ? FROM_FILENAME : FROM_EXIF;
                params.timeStampStartPos = cliParams.timestamp_startPos;

                VisApp visApp;
                string resultJson;
                FindLineResult result;

                params.resultImagePath.clear();
                for ( size_t i = 0; i < images.size(); ++i )
                {
                    if ( !result_folder.empty() )
                    {
                        params.resultImagePath = result_folder +
                                fs::path( images[ i ] ).stem().string() + "_result.png";
                    }
                    params.imagePath = images[ i ];
                    cout << fs::path( images[ i ] ).filename().string();
                    retVal = visApp.CalcLine( params, result );
                    cout << ( GC_OK == retVal ? ": SUCCESS" : " FAILURE" ) << endl;
                }
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
GC_STATUS FindWaterLevel( const Grime2CLIParams cliParams )
{
    FindLineParams params;

    params.imagePath = cliParams.src_imagePath;
    params.calibFilepath = cliParams.calib_jsonPath;
    params.resultImagePath = cliParams.result_imagePath;
    params.timeStampFormat = cliParams.timestamp_format;
    params.timeStampType = cliParams.timestamp_type == "from_filename" ? FROM_FILENAME : FROM_EXIF;
    params.timeStampStartPos = cliParams.timestamp_startPos;

    VisApp visApp;
    string resultJson;
    FindLineResult result;
    GC_STATUS retVal = visApp.CalcLine( params, result, resultJson );
    cout << ( GC_OK == retVal ? resultJson : "ERROR" ) << endl;
    return retVal;
}
GC_STATUS FormCalibJsonString( const Grime2CLIParams cliParams, string &json )
{
    GC_STATUS retVal = GC_OK;

    if ( true ) // bow tie calibration
    {
        json = "{\"calibType\": \"BowTie\", ";
        json += "\"calibWorldPt_csv\": \"" + cliParams.csvPath + "\", ";
        json += "\"stopSignFacetLength\": -1.0, ";
        json += "\"drawCalib\": 1, ";
        json += "\"drawMoveSearchROIs\": 1, ";
        json += "\"drawWaterLineSearchROI\": 1, ";
        json += "\"targetRoi_x\": \"" + to_string( cliParams.targetRoi_x ) + "\", ";
        json += "\"targetRoi_y\": \"" + to_string( cliParams.targetRoi_y ) + "\", ";
        json += "\"targetRoi_width\": \"" + to_string( cliParams.targetRoi_width ) + "\", ";
        json += "\"targetRoi_height\": \"" + to_string( cliParams.targetRoi_height ) + "\", ";
        json += "\"calibResult_json\": \"" + cliParams.calib_jsonPath + "\"}";
    }
    else if ( false ) // stop sign calibration
    {
        json = "{\"calibType\": \"StopSign\", ";
        if ( false ) // from file with world coords
        {
            json += "\"calibWorldPt_csv\": \"" + cliParams.csvPath + "\", ";
            json += "\"stopSignFacetLength\": -1.0, }";
            json += "\"drawCalib\": 1, ";
            json += "\"drawMoveSearchROIs\": 1, ";
            json += "\"drawWaterLineSearchROI\": 1, ";
            json += "\"targetRoi_x\": \"" + to_string( cliParams.targetRoi_x ) + "\", ";
            json += "\"targetRoi_y\": \"" + to_string( cliParams.targetRoi_y ) + "\", ";
            json += "\"targetRoi_width\": \"" + to_string( cliParams.targetRoi_width ) + "\", ";
            json += "\"targetRoi_height\": \"" + to_string( cliParams.targetRoi_height ) + "\", ";
            json += "\"calibResult_json\": \"" + cliParams.calib_jsonPath + "\"}";
        }
        else if ( false ) // from facet length
        {
            json += "\"calibWorldPt_csv\": \"\", ";
            json += "\"stopSignFacetLength\": " + to_string( 10.0 ) + ", ";
            json += "\"drawCalib\": 1, ";
            json += "\"drawMoveSearchROIs\": 1, ";
            json += "\"drawWaterLineSearchROI\": 1, ";
            json += "\"targetRoi_x\": \"" + to_string( cliParams.targetRoi_x ) + "\", ";
            json += "\"targetRoi_y\": \"" + to_string( cliParams.targetRoi_y ) + "\", ";
            json += "\"targetRoi_width\": \"" + to_string( cliParams.targetRoi_width ) + "\", ";
            json += "\"targetRoi_height\": \"" + to_string( cliParams.targetRoi_height ) + "\", ";
            json += "\"calibResult_json\": \"" + cliParams.calib_jsonPath + "\"}";
        }
        else
        {
            json.clear();
            FILE_LOG( logERROR ) << "No valid stop sign calibration method";
            retVal = GC_ERR;
        }
    }
    else
    {
        FILE_LOG( logERROR ) << "No calibration type selected";
        retVal = GC_ERR;
    }

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
            if ( ext == ".png" || ext == ".jpg" ||
                 ext == ".PNG" || ext == ".JPG" )
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
    cout << "ExifTool: "; fflush( stdout ); VisApp::GetExifToolVersion();
    cout << "   Boost: " << BOOST_VERSION / 100000 <<
            "." << BOOST_VERSION / 100 % 1000 <<
            "." << BOOST_VERSION % 100 << endl;
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
}
