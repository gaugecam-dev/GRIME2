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
#include <iostream>
#include <cstdlib>
#include <boost/bind/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/foreach.hpp>
#include <libexif/exif-ifd.h>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;
namespace pt = property_tree;

static ExifByteOrder FILE_BYTE_ORDER = EXIF_BYTE_ORDER_INTEL;

namespace gc
{

static void trim_spaces( char *buf );
static void show_tag( ExifData *d, ExifIfd ifd, ExifTag tag ); // Show the tag name and contents if the tag exists
static void show_mnote_tag( ExifData *d, unsigned tag ); // Show the given MakerNote tag if it exists
static ExifEntry *create_tag( ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len );
static ExifEntry *init_tag( ExifData *exif, ExifIfd ifd, ExifTag tag );

GC_STATUS MetaData::GetImageDescriptionExifData( const std::string imgFilepath, std::string &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        ExifData *ed = exif_data_new_from_file( imgFilepath.c_str() );
        if ( nullptr == ed )
        {
            FILE_LOG( logERROR ) << "[MetaData::GetImageDescriptionExifData]  File not readable or no EXIF data in file " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            retVal = GetExifTagString( ed, EXIF_TAG_IMAGE_DESCRIPTION, data );
#ifdef DEBUG_PNG_METADATA
            FILE_LOG( logINFO ) << "Key=EXIF_TAG_IMAGE_DESCRIPTION";
            FILE_LOG( logINFO ) << " Value=" << endl << data;
#endif
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetaData::GetMetadata] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetaData::GetMetadata( const std::string imgFilepath, std::string &jsonString )
{
    GC_STATUS retVal = GetImageDescriptionExifData( imgFilepath, jsonString );
    return retVal;
}
GC_STATUS MetaData::ReadLineFindResult( const std::string imgFilepath, FindData &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string exifData;
        GC_STATUS retVal = GetImageDescriptionExifData( imgFilepath, exifData );
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
            string cmdBuffer = "exiftool -ImageDescription=\"" + jsonString + "\" \"" + imgFilepath + "\"";
            cout << endl << cmdBuffer << endl;
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
GC_STATUS MetaData::GetExifTagString( const ExifData *exifData, const ExifTag tag, string &dataString )
{
    GC_STATUS retVal = GC_OK;
    ExifEntry *entry = exif_content_get_entry( exifData->ifd[ EXIF_IFD_0 ], tag );
    if ( nullptr == entry )
    {
        FILE_LOG( logERROR ) << "[MetaData::GetImageDescriptionExifData]  Could not retrieve data for tag=" << tag;
        retVal = GC_ERR;
    }
    else
    {
        char buf[32768];

        /* Get the contents of the tag in human-readable form */
        exif_entry_get_value( entry, buf, sizeof( buf ) );

        /* Don't bother printing it if it's entirely blank */
        trim_spaces( buf );

        dataString = string( buf );
    }
    return retVal;
}
GC_STATUS MetaData::Retrieve( const std::string filepath, std::string &data, ExifFeatures &exifFeat )
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
            ExifData *ed = exif_data_new_from_file( filepath.c_str() );
            if ( nullptr == ed )
            {
                FILE_LOG( logERROR ) << "[]  File not readable or no EXIF data in file " << filepath;
                retVal = GC_ERR;
            }
            else
            {
                string dataString;
                retVal = GetExifTagString( ed, EXIF_TAG_IMAGE_WIDTH, dataString );
                if ( GC_OK == retVal )
                {
                    exifFeat.imageDims.width = stoi( dataString );
                    retVal = GetExifTagString( ed, EXIF_TAG_IMAGE_LENGTH, dataString );
                    if ( GC_OK == retVal )
                    {
                        exifFeat.imageDims.height = stoi( dataString );
                        retVal = GetExifTagString( ed, EXIF_TAG_DATE_TIME_ORIGINAL, dataString );
                        if ( GC_OK == retVal )
                        {
                            exifFeat.captureTime = ConvertToLocalTimestamp( dataString );
                            retVal = GetExifTagString( ed, EXIF_TAG_FNUMBER, dataString );
                            if ( GC_OK == retVal )
                            {
                                exifFeat.fNumber = stod( dataString );
                                retVal = GetExifTagString( ed, EXIF_TAG_EXPOSURE_TIME, dataString );
                                if ( GC_OK == retVal )
                                {
                                    exifFeat.exposureTime = stod( dataString );
                                    retVal = GetExifTagString( ed, EXIF_TAG_SHUTTER_SPEED_VALUE, dataString );
                                    if ( GC_OK == retVal )
                                    {
                                        exifFeat.shutterSpeed = stod( dataString );
                                        retVal = GetExifTagString( ed, EXIF_TAG_ISO_SPEED_RATINGS, dataString );
                                        if ( GC_OK == retVal )
                                        {
                                            exifFeat.isoSpeedRating = stoi( dataString );
                                        }
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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// libexif calls
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void trim_spaces(char *buf)
{
    char *s = buf-1;
    for (; *buf; ++buf) {
        if (*buf != ' ')
            s = buf;
    }
    *++s = 0; /* nul terminate the string on the first of the final spaces */
}

/* Show the tag name and contents if the tag exists */
void show_tag(ExifData *d, ExifIfd ifd, ExifTag tag)
{
    /* See if this tag exists */
    ExifEntry *entry = exif_content_get_entry(d->ifd[ifd],tag);
    if (entry) {
        char buf[32768];

        /* Get the contents of the tag in human-readable form */
        exif_entry_get_value(entry, buf, sizeof(buf));

        /* Don't bother printing it if it's entirely blank */
        trim_spaces(buf);
        if (*buf) {
            cout << exif_tag_get_name_in_ifd(tag,ifd) << "[Size=" << sizeof(buf) << "]" << endl;
            cout << buf << endl;
        }
    }
}

/* Show the given MakerNote tag if it exists */
void show_mnote_tag(ExifData *d, unsigned tag)
{
    ExifMnoteData *mn = exif_data_get_mnote_data(d);
    if (mn) {
        int num = exif_mnote_data_count(mn);
        int i;

        /* Loop through all MakerNote tags, searching for the desired one */
        for (i=0; i < num; ++i) {
            char buf[1024];
            if (exif_mnote_data_get_id(mn, i) == tag) {
                if (exif_mnote_data_get_value(mn, i, buf, sizeof(buf))) {
                    /* Don't bother printing it if it's entirely blank */
                    trim_spaces(buf);
                    if (*buf) {
                        printf("%s: %s\n", exif_mnote_data_get_title(mn, i),
                            buf);
                    }
                }
            }
        }
    }
}
/* Get an existing tag, or create one if it doesn't exist */
ExifEntry *init_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
    ExifEntry *entry;
    /* Return an existing tag if one exists */
    if (!((entry = exif_content_get_entry (exif->ifd[ifd], tag)))) {
        /* Allocate a new entry */
        entry = exif_entry_new ();
        assert(entry != NULL); /* catch an out of memory condition */
        entry->tag = tag; /* tag must be set before calling
                 exif_content_add_entry */

        /* Attach the ExifEntry to an IFD */
        exif_content_add_entry (exif->ifd[ifd], entry);

        /* Allocate memory for the entry and fill with default data */
        exif_entry_initialize (entry, tag);

        /* Ownership of the ExifEntry has now been passed to the IFD.
         * One must be very careful in accessing a structure after
         * unref'ing it; in this case, we know "entry" won't be freed
         * because the reference count was bumped when it was added to
         * the IFD.
         */
        exif_entry_unref(entry);
    }
    return entry;
}

/* Create a brand-new tag with a data field of the given length, in the
 * given IFD. This is needed when exif_entry_initialize() isn't able to create
 * this type of tag itself, or the default data length it creates isn't the
 * correct length.
 */
ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len)
{
    void *buf;
    ExifEntry *entry;

    /* Create a memory allocator to manage this ExifEntry */
    ExifMem *mem = exif_mem_new_default();
    assert(mem != NULL); /* catch an out of memory condition */

    /* Create a new ExifEntry using our allocator */
    entry = exif_entry_new_mem (mem);
    assert(entry != NULL);

    /* Allocate memory to use for holding the tag data */
    buf = exif_mem_alloc( mem, len );
    assert(buf != NULL);

    /* Fill in the entry */
    entry->data = reinterpret_cast< unsigned char * >( buf );
    entry->size = len;
    entry->tag = tag;
    entry->components = len;
    entry->format = EXIF_FORMAT_UNDEFINED;

    /* Attach the ExifEntry to an IFD */
    exif_content_add_entry (exif->ifd[ifd], entry);

    /* The ExifMem and ExifEntry are now owned elsewhere */
    exif_mem_unref(mem);
    exif_entry_unref(entry);

    return entry;
}


} // namespace gc
