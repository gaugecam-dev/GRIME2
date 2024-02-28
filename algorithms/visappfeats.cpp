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
#include "visappfeats.h"
#include <limits>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "timestampconvert.h"
#include "metadata.h"
#include "csvreader.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;

#ifdef DEBUG_VISAPP_FEATURES
#undef DEBUG_VISAPP_FEATURES
#ifdef WIN32
static string DEBUG_FOLDER = "c:/gaugecam/debug/visappfeats/";
#else
static string DEBUG_FOLDER = "/var/tmp/gaugecam/visappfeats/";
#endif
#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// static helper functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static bool is_number( const string& s )
{
    return !s.empty() && find_if( s.begin(), s.end(), [](char c) { return !( isdigit( c ) || c == '.' || c== '-' ); } ) == s.end();
}
static double DISTANCE( const Point2d a, const Point2d b )
{
    return static_cast< double >( sqrt( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) ) );
}



namespace gc
{

VisAppFeats::VisAppFeats()
{
#ifdef DEBUG_VISAPP_FEATURES
    if ( !boost::filesystem::exists( DEBUG_FOLDER ) )
    {
        bool bOk = boost::filesystem::create_directories( DEBUG_FOLDER );
        if ( bOk )
        {
            FILE_LOG( logINFO ) << "Create find visappfeat debug folder: " << DEBUG_FOLDER;
        }
        else
        {
            FILE_LOG( logWARNING ) << "Could NOT create find visappfeat debug folder: " << DEBUG_FOLDER;
        }
    }
#endif
}
GC_STATUS VisAppFeats::CreateCSVFileAndHeader( const string filepath, const FeatureSet &featSet )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ofstream outFile;
        if ( !fs::exists( filepath ) )
        {
            if ( !fs::exists( fs::path( filepath ).parent_path() ) )
            {
                 bool bRet = fs::create_directories( fs::path( filepath ).parent_path() );
                 if ( !bRet )
                 {
                     FILE_LOG( logERROR ) << "[VisAppFeats::CreateCSVFileAndHeader] Could not create folder for CSV file: " << filepath;
                     retVal = GC_ERR;
                 }
            }
            if ( GC_OK == retVal )
            {
                outFile.open( filepath, ios_base::out );
                if ( !outFile.is_open() )
                {
                    FILE_LOG( logERROR ) << "[VisAppFeats::CreateCSVFileAndHeader] Could not create CSV: " << filepath;
                    retVal = GC_ERR;
                }
                else
                {
                    outFile << "SensorTime, CaptureTime, Filename, Agency, SiteNumber, TimeZone, Stage, Discharge, ";
                    outFile << "CalcTimestamp, width, height, exposure, fNumber, isoSpeed, shutterSpeed, areaFeatCount";
                    if ( 0 < featSet.areaFeats.size() )
                    {
                        outFile << ", ";
                        for ( size_t i = 0; i < featSet.areaFeats.size(); ++i )
                        {
                            outFile << "grayMean " << i << ", graySigma " << i << ", ";
                            outFile << "entropyMean " << i << ", entropySigma " << i << ", ";
                            outFile << "hMean " << i << ", hSigma " << i << ", ";
                            outFile << "sMean " << i << ", sSigma " << i << ", ";
                            outFile << "vMean " << i << ", vSigma " << i;
                            if ( featSet.areaFeats.size() - 1 > i )
                                outFile << ", ";
                        }
                    }
                    outFile << endl;
                }
                outFile.close();
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisAppFeats::CreateCSVFileAndHeader] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisAppFeats::ParseRow( const vector< string > data, FeatureSet &feat )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        int j = 0;
        PixelStats pixStats;

        feat.clear();
        feat.exif.captureTime = data[ j++ ];
        feat.imageFilename = data[ j++ ];

        feat.calcTimestamp = data[ j++ ];
        feat.imageSize.width = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.imageSize.height = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.exif.exposureTime = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.exif.fNumber = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.exif.isoSpeedRating = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.exif.shutterSpeed = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

        ImageAreaFeatures areaFeat;
        int areaFeatCount  = is_number( data[ j ] ) ? stoi( data[ j ] ) : 0; ++j;
        for ( size_t i = 0; i < static_cast< size_t >( areaFeatCount ); ++i )
        {
            areaFeat.clear();

            areaFeat.grayStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.grayStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

            areaFeat.entropyStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.entropyStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            feat.areaFeats.push_back( areaFeat );
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisAppFeats::ReadCSV] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisAppFeats::ReadCSV( const string filepath, vector< FeatureSet > &featSets )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        CSVReader reader( filepath );
        vector< vector< string > > data = reader.getData();
        if ( data.empty() )
        {
            FILE_LOG( logERROR ) << "[VisAppFeats::ReadCSV] No data in file " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            FeatureSet feat;
            size_t count = data[ 0 ].size();
            for ( size_t i = 1; i < data.size(); ++i )
            {
                // cout << "Parse features: " << i << " of " << data.size() << "\r";
                if ( count <= data[ i ].size() )
                {
                    retVal = ParseRow( data[ i ], feat );
                    if ( GC_OK == retVal )
                    {
                        featSets.push_back( feat );
                    }
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisAppFeats::ReadCSV] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisAppFeats::CalcMovement( const cv::Mat img, cv::Point &ptOrig, cv::Point &ptMove, double &angle )
{
    GC_STATUS retVal = anchor.CalcMoveModel( img, ptOrig, ptMove, angle );
    return retVal;
}
GC_STATUS VisAppFeats::SetAnchorRef( const string imgRefFilepath, const cv::Rect rect )
{
    GC_STATUS retVal = anchor.SetRef( imgRefFilepath, rect );
    return retVal;
}
GC_STATUS VisAppFeats::ReadSettings( const std::string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisAppFeats::ReadSettings] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS VisAppFeats::WriteSettings( const std::string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ofstream settings( jsonFilepath );
        if ( !settings.is_open() )
        {
            FILE_LOG( logERROR ) << "[VisAppFeats::WriteSettings] Could not open file for writing -- " << jsonFilepath;
            retVal = GC_ERR;
        }
        else
        {
            Rect anchorRect = anchor.ModelRect();
            settings << "{" << endl;
            settings << "  \"anchor_model_roi_x\": " << anchorRect.x << "," << endl;
            settings << "  \"anchor_model_roi_y\": " << anchorRect.y << "," << endl;
            settings << "  \"anchor_model_roi_width\": " << anchorRect.width << "," << endl;
            settings << "  \"anchor_model_roi_height\": " << anchorRect.height << "," << endl;
            settings << "  \"anchor_model_ref_image\": " << anchor.ModelRefImagePath() << "," << endl;
            settings << "  \"feature_params\": " << endl;
            settings << "  {" << endl;
            settings << "    \"timestamp_format\": \"" << featCalcParams.timeStampFormat << "\"," << endl;
            settings << "    \"timestamp_length\": " << featCalcParams.timeStampLength << "," << endl;
            settings << "    \"timestamp_start_pos\": " << featCalcParams.timeStampStartPos << "," << endl;
            settings << "    \"timestamp_type\": " << featCalcParams.timeStampType << "," << endl;
            settings << "    \"area_rois\": " << endl;
            settings << "    [" << endl;

            for ( size_t i = 0; i < featCalcParams.areaROIs.size(); ++i )
            {
                settings << "      {" << endl;
                settings << "        \"color_blue\":" << static_cast< int >( featCalcParams.areaROIs[ i ].color.val[ 0 ] ) << "," << endl;
                settings << "        \"color_green\":" << static_cast< int >( featCalcParams.areaROIs[ i ].color.val[ 1 ] ) << "," << endl;
                settings << "        \"color_red\":" << static_cast< int >( featCalcParams.areaROIs[ i ].color.val[ 2 ] ) << "," << endl;
                settings << "        \"name\": \"" << featCalcParams.areaROIs[ i ].name << "\"," << endl;
                settings << "        \"roi_type\": \"" << featCalcParams.areaROIs[ i ].roi_type << "\"," << endl;
                settings << "        \"ellipse_center_x\":" << featCalcParams.areaROIs[ i ].rotRect.center.x << "," << endl;
                settings << "        \"ellipse_center_y\":" << featCalcParams.areaROIs[ i ].rotRect.center.y << "," << endl;
                settings << "        \"ellipse_width\":" << featCalcParams.areaROIs[ i ].rotRect.size.width << "," << endl;
                settings << "        \"ellipse_height\":" << featCalcParams.areaROIs[ i ].rotRect.size.height << "," << endl;
                settings << "        \"ellipse_angle\":" << featCalcParams.areaROIs[ i ].rotRect.angle << "," << endl;
                settings << "        \"contour_pts\": " << endl;
                settings << "        [";

                for ( size_t j = 0; j < featCalcParams.areaROIs[ i ].contour.size(); ++j )
                {
                    settings << featCalcParams.areaROIs[ i ].contour[ j ];
                    if ( featCalcParams.areaROIs[ i ].contour.size() - 1 > i )
                    {
                        settings << ",";
                    }
                }
                settings << "]" << endl;
                settings << "      }" << endl;
                if ( featCalcParams.areaROIs.size() - 1 > i )
                {
                    settings << ",";
                }
                settings << endl;
            }

            settings << "    ]" << endl;
            settings << "  }" << endl;
            settings << "}" << endl;
        }
        settings.close();
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisAppFeats::WriteSettings] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
