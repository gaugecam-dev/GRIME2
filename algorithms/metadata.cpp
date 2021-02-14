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
#include "metadata.h"
#include <stdio.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <boost/bind/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/foreach.hpp>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;
namespace pt = property_tree;

namespace gc
{

GC_STATUS MetaData::GetExifData( const std::string filepath, const std::string tag, std::string &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string cmdStr = "exiftool -q -" + tag + " \"" + filepath + "\"";
        FILE *cmd = popen( cmdStr.c_str(), "r" );
        if ( nullptr == cmd )
        {
            FILE_LOG( logERROR ) << "[MetaData::GetExifData] Could not open file to retrieve metadata: " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            char buffer[ 256 ];
            while( nullptr != fgets( buffer, sizeof( buffer ), cmd ) )
            {
                data += buffer;
            }
            pclose( cmd );
        }

        cout << endl << data << endl;

#ifdef DEBUG_PNG_METADATA
        FILE_LOG( logINFO ) << "Key=EXIF_TAG_IMAGE_DESCRIPTION";
        FILE_LOG( logINFO ) << " Value=" << endl << data;
#endif
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::GetExifData] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::GetMetadata( const std::string imgFilepath, std::string &jsonString )
{
    GC_STATUS retVal = GetExifData( imgFilepath, "ImageDescription", jsonString );
    return retVal;
}
GC_STATUS MetaData::ReadLineFindResult( const std::string imgFilepath, FindData &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string exifData;
        GC_STATUS retVal = GetExifData( imgFilepath, "ImageDescription", exifData );
        retVal = ParseFindData( exifData, data );
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::ReadLineFindResult] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::WriteLineFindResult( const std::string imgFilepath, const FindData data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string jsonString;
        retVal = FindResultToJsonString( data, jsonString );
        if ( GC_OK == retVal )
        {
            string cmdBuffer = "exiftool -overwrite_original -ImageDescription=\"" + jsonString + "\" \"" + imgFilepath + "\"";
            // cout << endl << cmdBuffer << endl;
            std::system( cmdBuffer.c_str() );
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::WriteLineFindResult] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::ParseFindPointSetString( const pt::ptree &child, const std::string key, FindPointSet &ptSet )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        ptSet.clear();

        ptSet.lftPixel.x = child.get< double >( key + "-lftPixel-x", -1.0 );
        ptSet.lftPixel.y = child.get< double >( key + "-lftPixel-y", -1.0 );
        ptSet.ctrPixel.x = child.get< double >( key + "-ctrPixel-x", -1.0 );
        ptSet.ctrPixel.y = child.get< double >( key + "-ctrPixel-y", -1.0 );
        ptSet.rgtPixel.x = child.get< double >( key + "-rgtPixel-x", -1.0 );
        ptSet.rgtPixel.y = child.get< double >( key + "-rgtPixel-y", -1.0 );

        ptSet.lftWorld.x = child.get< double >( key + "-lftWorld-x", -1.0 );
        ptSet.lftWorld.y = child.get< double >( key + "-lftWorld-y", -1.0 );
        ptSet.ctrWorld.x = child.get< double >( key + "-ctrWorld-x", -1.0 );
        ptSet.ctrWorld.y = child.get< double >( key + "-ctrWorld-y", -1.0 );
        ptSet.rgtWorld.x = child.get< double >( key + "-rgtWorld-x", -1.0 );
        ptSet.rgtWorld.y = child.get< double >( key + "-rgtWorld-y", -1.0 );

        ptSet.angleWorld = child.get< double >( key + "-angle", -1.0 );
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::ParseFindPointSetString] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::ParseFindData( const std::string &metadata, FindData &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        data.clear();

        stringstream ss;
        ss << metadata;

        pt::ptree top_level, child, subchild;
        pt::json_parser::read_json( ss, top_level );
        data.findlineParams.imagePath = top_level.get< std::string >( "imagePath", "N/A" );
        data.findlineParams.datetimeOriginal = top_level.get< std::string >( "datetimeOriginal", "1955-09-24T12:05:00" );
        data.findlineParams.resultImagePath = top_level.get< std::string >( "resultImagePath", "N/A" );
        data.findlineParams.datetimeProcessing = top_level.get< std::string >( "datetimeProcessing", "1955-09-24T12:05:00" );
        data.findlineParams.calibFilepath = top_level.get< std::string >( "calibFilepath", "N/A" );
        data.calibSettings.gridSize.width = top_level.get< int >( "calibSettings-gridSize-width", -1 );
        data.calibSettings.gridSize.height = top_level.get< int >( "calibSettings-gridSize-height", -1 );
        data.calibSettings.moveSearchRegionLft.x = top_level.get< int >( "calibSettings-moveSearchRegionLft-x", -1 );
        data.calibSettings.moveSearchRegionLft.y = top_level.get< int >( "calibSettings-moveSearchRegionLft-y", -1 );
        data.calibSettings.moveSearchRegionLft.width = top_level.get< int >( "calibSettings-moveSearchRegionLft-width", -1 );
        data.calibSettings.moveSearchRegionLft.height = top_level.get< int >( "calibSettings-moveSearchRegionLft-height", -1 );
        data.calibSettings.moveSearchRegionRgt.x = top_level.get< int >( "calibSettings-moveSearchRegionRgt-x", -1 );
        data.calibSettings.moveSearchRegionRgt.y = top_level.get< int >( "calibSettings-moveSearchRegionRgt-y", -1 );
        data.calibSettings.moveSearchRegionRgt.width = top_level.get< int >( "calibSettings-moveSearchRegionRgt-width", -1 );
        data.calibSettings.moveSearchRegionRgt.height = top_level.get< int >( "calibSettings-moveSearchRegionRgt-height", -1 );

        Point2d pt2d;
        child = top_level.get_child( "calibSettings-pixelPoints" );
        BOOST_FOREACH( const pt::ptree::value_type &v, child )
        {
            pt2d.x = v.second.get< double >( "x", - 1.0 );
            pt2d.y = v.second.get< double >( "y", - 1.0 );
            data.calibSettings.pixelPoints.push_back( pt2d );
        }

        child = top_level.get_child( "calibSettings-worldPoints" );
        BOOST_FOREACH( const pt::ptree::value_type &v, child )
        {
            pt2d.x = v.second.get< double >( "x", - 1.0 );
            pt2d.y = v.second.get< double >( "y", - 1.0 );
            data.calibSettings.worldPoints.push_back( pt2d );
        }

        LineEnds ends;
        child = top_level.get_child( "calibSettings-searchLines" );
        BOOST_FOREACH( const pt::ptree::value_type &v, child )
        {
            ends.top.x = v.second.get< int >( "top-x", - 1 );
            ends.top.y = v.second.get< int >( "top-y", - 1 );
            ends.bot.x = v.second.get< int >( "bot-x", - 1 );
            ends.bot.y = v.second.get< int >( "bot-y", - 1 );
            data.calibSettings.searchLines.push_back( ends );
        }

        child = top_level.get_child( "findlineResult" );
        data.findlineResult.findSuccess = child.get< bool >( "data-findLineResult-findSuccess", false );
        retVal = ParseFindPointSetString( child, "data-findlineResult-refMovePts", data.findlineResult.refMovePts );
        if ( GC_OK == retVal )
        {
            retVal = ParseFindPointSetString( child, "data-findlineResult-calcLinePts", data.findlineResult.calcLinePts );
            if ( GC_OK == retVal )
            {
                retVal = ParseFindPointSetString( child, "data-findlineResult-foundMovePts", data.findlineResult.foundMovePts );
                if ( GC_OK == retVal )
                {
                    retVal = ParseFindPointSetString( child, "data-findlineResult-offsetMovePts", data.findlineResult.offsetMovePts );
                    if ( GC_OK == retVal )
                    {
                        subchild = child.get_child( "data-findlineResult-foundPoints" );
                        BOOST_FOREACH( const pt::ptree::value_type &v, subchild )
                        {
                            pt2d.x = v.second.get< double >( "x", - 1.0 );
                            pt2d.y = v.second.get< double >( "y", - 1.0 );
                            data.findlineResult.foundPoints.push_back( pt2d );
                        }
                    }
                }
            }
        }
        data.findlineResult.waterLevelAdjusted.x = data.findlineResult.calcLinePts.ctrWorld.x - data.findlineResult.offsetMovePts.ctrWorld.x;
        data.findlineResult.waterLevelAdjusted.y = data.findlineResult.calcLinePts.ctrWorld.y - data.findlineResult.offsetMovePts.ctrWorld.y;
        data.findlineResult.calcLinePts.angleWorld = atan( ( data.findlineResult.calcLinePts.rgtWorld.y - data.findlineResult.calcLinePts.lftWorld.y ) /
                                                      ( data.findlineResult.calcLinePts.rgtWorld.x - data.findlineResult.calcLinePts.lftWorld.x ) ) * 180 / 3.14159265;

        std::string buffer;
        subchild = child.get_child( "data-findlineResult-msgs" );
        BOOST_FOREACH( const pt::ptree::value_type &v, subchild )
        {
            buffer = v.second.get_value< std::string >( "N/A" );
            data.findlineResult.msgs.push_back( buffer );
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::ParseFindData] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::CreateFindPointSetString( const FindPointSet set, const std::string key, std::string &jsonString )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        stringstream ss;
        ss << fixed << setprecision( 3 );
        ss << "      \\\"" << key << "-lftPixel-x\\\": " << set.lftPixel.x << ",";
        ss << "      \\\"" << key << "-lftPixel-y\\\": " << set.lftPixel.y << ",";
        ss << "      \\\"" << key << "-ctrPixel-x\\\": " << set.ctrPixel.x << ",";
        ss << "      \\\"" << key << "-ctrPixel-y\\\": " << set.ctrPixel.y << ",";
        ss << "      \\\"" << key << "-rgtPixel-x\\\": " << set.rgtPixel.x << ",";
        ss << "      \\\"" << key << "-rgtPixel-y\\\": " << set.rgtPixel.y << ",";
        ss << "      \\\"" << key << "-lftWorld-x\\\": " << set.lftWorld.x << ",";
        ss << "      \\\"" << key << "-lftWorld-y\\\": " << set.lftWorld.y << ",";
        ss << "      \\\"" << key << "-ctrWorld-x\\\": " << set.ctrWorld.x << ",";
        ss << "      \\\"" << key << "-ctrWorld-y\\\": " << set.ctrWorld.y << ",";
        ss << "      \\\"" << key << "-rgtWorld-x\\\": " << set.rgtWorld.x << ",";
        ss << "      \\\"" << key << "-rgtWorld-y\\\": " << set.rgtWorld.y << ",";
        ss << "      \\\"" << key << "-angle\\\": " << set.angleWorld;
        jsonString = ss.str();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::CreateFindPointSetString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::FindResultToJsonString( const FindData data, std::string &jsonString )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        stringstream ss;
        ss << "{";
        ss << "   \\\"imagePath\\\": \\\"" << data.findlineParams.imagePath << "\\\",";
        ss << "   \\\"datetimeOriginal\\\": \\\"" << data.findlineParams.datetimeOriginal << "\\\",";
        ss << "   \\\"resultImagePath\\\": \\\"" << data.findlineParams.resultImagePath << "\\\",";
        ss << "   \\\"datetimeProcessing\\\": \\\"" << data.findlineParams.datetimeProcessing << "\\\",";
        ss << "   \\\"calibFilepath\\\": \\\"" << data.findlineParams.calibFilepath << "\\\",";
        ss << "   \\\"calibSettings-gridSize-width\\\": " << data.calibSettings.gridSize.width << ",";
        ss << "   \\\"calibSettings-gridSize-height\\\": " << data.calibSettings.gridSize.height << ",";
        ss << "   \\\"calibSettings-moveSearchRegionLft-x\\\": " << data.calibSettings.moveSearchRegionLft.x << ",";
        ss << "   \\\"calibSettings-moveSearchRegionLft-y\\\": " << data.calibSettings.moveSearchRegionLft.y << ",";
        ss << "   \\\"calibSettings-moveSearchRegionLft-width\\\": " << data.calibSettings.moveSearchRegionLft.width << ",";
        ss << "   \\\"calibSettings-moveSearchRegionLft-height\\\": " << data.calibSettings.moveSearchRegionLft.height << ",";
        ss << "   \\\"calibSettings-moveSearchRegionRgt-x\\\": " << data.calibSettings.moveSearchRegionRgt.x << ",";
        ss << "   \\\"calibSettings-moveSearchRegionRgt-y\\\": " << data.calibSettings.moveSearchRegionRgt.y << ",";
        ss << "   \\\"calibSettings-moveSearchRegionRgt-width\\\": " << data.calibSettings.moveSearchRegionRgt.width << ",";
        ss << "   \\\"calibSettings-moveSearchRegionRgt-height\\\": " << data.calibSettings.moveSearchRegionRgt.height << ",";
        ss << "   \\\"calibSettings-pixelPoints\\\": [";
        ss << fixed << setprecision( 3 );
        for ( size_t i = 0; i < data.calibSettings.pixelPoints.size(); ++i )
        {
            ss << "      {\\\"x\\\": " << data.calibSettings.pixelPoints[ i ].x << ", \\\"y\\\": " << data.calibSettings.pixelPoints[ i ].y << "}";
            if ( data.calibSettings.pixelPoints.size() - 1 > i )
                ss << ",";
        }
        ss << "   ],";
        ss << "   \\\"calibSettings-worldPoints\\\": [";
        for ( size_t i = 0; i < data.calibSettings.worldPoints.size(); ++i )
        {
            ss << "      {\\\"x\\\": " << data.calibSettings.worldPoints[ i ].x << ", \\\"y\\\": " << data.calibSettings.worldPoints[ i ].y << "}";
            if ( data.calibSettings.worldPoints.size() - 1 > i )
                ss << ",";
        }
        ss << "   ],";
        ss << "   \\\"calibSettings-searchLines\\\": [";
        ss << fixed << setprecision( 0 );
        for ( size_t i = 0; i < data.calibSettings.searchLines.size(); ++i )
        {
            ss << "      { \\\"top-x\\\": " << data.calibSettings.searchLines[ i ].top.x << ", ";
            ss << "\\\"top-y\\\": " << data.calibSettings.searchLines[ i ].top.y << ", ";
            ss << "\\\"bot-x\\\": " << data.calibSettings.searchLines[ i ].bot.x << ", ";
            ss << "\\\"bot-y\\\": " << data.calibSettings.searchLines[ i ].bot.y << " }";
            if ( data.calibSettings.searchLines.size() - 1 > i )
                ss << ",";
        }
        ss << "   ],";
        ss << "   \\\"findlineResult\\\": {";
        ss << "      \\\"data-findLineResult-findSuccess\\\": " << ( data.findlineResult.findSuccess ? std::string( "true" ) : std::string( "false" ) ) << ",";

        std::string buffer;
        retVal = CreateFindPointSetString( data.findlineResult.refMovePts, "data-findlineResult-refMovePts", buffer ); ss << buffer << ",";
        if ( GC_OK == retVal )
        {
            retVal = CreateFindPointSetString( data.findlineResult.calcLinePts, "data-findlineResult-calcLinePts", buffer ); ss << buffer << ",";
            if ( GC_OK == retVal )
            {
                retVal = CreateFindPointSetString( data.findlineResult.foundMovePts, "data-findlineResult-foundMovePts", buffer ); ss << buffer << ",";
                if ( GC_OK == retVal )
                {
                    retVal = CreateFindPointSetString( data.findlineResult.offsetMovePts, "data-findlineResult-offsetMovePts", buffer ); ss << buffer << ",";
                }
            }
        }

        if ( GC_OK == retVal )
        {
            ss << fixed << setprecision( 3 );
            ss << "      \\\"data-findlineResult-foundPoints\\\": [";
            for ( size_t i = 0; i < data.findlineResult.foundPoints.size(); ++i )
            {
                ss << "         {\\\"x\\\": " << data.findlineResult.foundPoints[ i ].x << ", \\\"y\\\": " << data.findlineResult.foundPoints[ i ].y << "}";
                if ( data.findlineResult.foundPoints.size() - 1 > i )
                    ss << ",";
            }
            ss << "      ],";
            ss << "      \\\"data-findlineResult-msgs\\\": [";
            for ( size_t i = 0; i < data.findlineResult.msgs.size(); ++i )
            {
                ss << "         \\\"" << data.findlineResult.msgs[ i ] << "\\\"";
                if ( data.findlineResult.msgs.size() - 1 > i )
                    ss << ",";
            }
            ss << "      ]";
            ss << "   }";
            ss << "}";
            jsonString = ss.str();
        }
        else
        {
            jsonString = "FAIL: Could not create json string";
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::FindResultToJsonString] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// exifTimestamp example: 2012:09:30 15:38:49
// isoTimeStamp example:  2019-09-15T20:08:12
std::string MetaData::ConvertToLocalTimestamp( const std::string exifTimestamp )
{
    std::string isoTimestamp;
    try
    {
        if ( 19 != exifTimestamp.size() )
        {
            isoTimestamp = "Invalid length exif timestamp " + exifTimestamp;
            FILE_LOG( logERROR ) << "[ExifMetadata::ConvertToISOTimestamp] " << "Invalid exif timestamp " + exifTimestamp;
        }
        else
        {
            if ( exifTimestamp[ 4 ] != ':' || exifTimestamp[ 7 ] != ':' ||
                 exifTimestamp[ 10 ] != ' ' || exifTimestamp[ 13 ] != ':' || exifTimestamp[ 16 ] != ':' )
            {
                isoTimestamp = "Invalid exif timestamp " + exifTimestamp;
                FILE_LOG( logERROR ) << "[ExifMetadata::ConvertToISOTimestamp] " << "Invalid exif timestamp " + exifTimestamp;
            }
            else
            {
                isoTimestamp = exifTimestamp;
                isoTimestamp[ 4 ] = '-';
                isoTimestamp[ 7 ] = '-';
                isoTimestamp[ 10 ] = 'T';
            }
        }
    }
    catch( std::exception &e )
    {
        isoTimestamp = "Could not convert " + exifTimestamp + " to ISO timestamp";
        FILE_LOG( logERROR ) << "[ExifMetadata::ConvertToISOTimestamp] " << e.what();
    }
    return isoTimestamp;
}
GC_STATUS MetaData::Retrieve( const std::string filepath, ExifFeatures &exifFeat )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !fs::exists( filepath ) )
        {
            FILE_LOG( logERROR ) << "[ExifMetadata::Retrieve] Image file does not exist: " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            string data;
            retVal = GetExifData( filepath, "ImageWidth", data );
            if ( GC_OK == retVal )
            {
                exifFeat.imageDims.width = stoi( data );
                retVal = GetExifData( filepath, "ImageHeight", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.imageDims.height = stoi( data );
                    retVal = GetExifData( filepath, "Date/Time Original", data );
                    if ( GC_OK == retVal )
                    {
                        exifFeat.captureTime = ConvertToLocalTimestamp( data );
                        retVal = GetExifData( filepath, "FNumber", data );
                        if ( GC_OK == retVal )
                        {
                            exifFeat.fNumber = stod( data );
                            retVal = GetExifData( filepath, "ExposureTime", data );
                            if ( GC_OK == retVal )
                            {
                                exifFeat.exposureTime = stod( data );
                                retVal = GetExifData( filepath, "ShutterSpeed", data );
                                if ( GC_OK == retVal )
                                {
                                    exifFeat.shutterSpeed = stod( data );
                                    retVal = GetExifData( filepath, "ISO", data );
                                    if ( GC_OK == retVal )
                                    {
                                        exifFeat.isoSpeedRating = stoi( data );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[ExifMetadata::Retrieve] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
