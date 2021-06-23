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
#include "arghandler.h"
#include "../algorithms/visapp.h"

using namespace std;
using namespace gc;

// example command lines
// --version
// --show_help
// --calibrate "/home/kchapman/repos/GRIME2/gcgui/config/2012_demo/06/NRmarshDN-12-06-45-10-30.jpg" --csv_file "/home/kchapman/repos/GRIME2/gcgui/config/calibration_target_world_coordinates.csv" --result_image "/home/kchapman/Desktop/calib/calib_result.png"
// --show_metadata "/home/kchapman/data/idaho_power/bad_cal_bad_line_find/TREK0003.jpg"
// --find_line --timestamp_from_filename --timestamp_start_pos 10 --timestamp_length 14 --timestamp_format "yy-mm-dd-HH-MM" "/home/kchapman/repos/GRIME2/gcgui/config/2012_demo/06/NRmarshDN-12-06-30-10-45.jpg" --calib_json "/home/kchapman/Desktop/calib/calib.json" --result_image "/home/kchapman/Desktop/calib/find_line_result.png"
// --run_folder --timestamp_from_filename --timestamp_start_pos 10 --timestamp_length 14 --timestamp_format "yy-mm-dd-HH-MM" "/home/kchapman/repos/GRIME2/gcgui/config/2012_demo/06/" --calib_json "/home/kchapman/Desktop/calib/calib.json" --csv_file "/home/kchapman/Desktop/calib/" --result_folder "/home/kchapman/Desktop/calib/find_line_folder.csv"

// forward declarations
void ShowVersion();
GC_STATUS FindWaterLevel( const Grime2CLIParams cliParams );
GC_STATUS RunFolder( const Grime2CLIParams cliParams );

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
                VisApp vis;
                retVal = vis.Calibrate( params.src_imagePath, params.csvPath, params.calib_jsonPath, params.result_imagePath );
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
                VisApp vis;
                retVal = vis.CreateAnimation( params.src_imagePath, params.result_imagePath, params.fps, params.scale );
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
