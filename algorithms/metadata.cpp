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

// forward declarations
string &trim( string &str );
string &ltrim( string &str );
string &rtrim( string &str );
string trim_copy( string const &str );

namespace gc
{
GC_STATUS MetaData::GetExifData( const string filepath, const string tag, string &data )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string cmdStr = "exiftool -q -" + tag + " \"" + filepath + "\"";
#ifdef _WIN32
        FILE *cmd = _popen( cmdStr.c_str(), "r" );
#else
        FILE *cmd = popen( cmdStr.c_str(), "r" );
#endif
        if ( nullptr == cmd )
        {
            FILE_LOG( logERROR ) << "[MetaData::GetExifData] Could not open file to retrieve metadata: " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            string strBuf;
            char buffer[ 256 ];
            while( nullptr != fgets( buffer, sizeof( buffer ), cmd ) )
            {
                strBuf += buffer;
            }
#ifdef _WIN32
            _pclose( cmd );
#else
            pclose( cmd );
#endif
            size_t pos = strBuf.find( ":" );
            if ( string::npos == pos )
            {
                FILE_LOG( logERROR ) << "[MetaData::GetExifData] Invalid exif data (no \":\" found: " << strBuf;
                retVal = GC_ERR;
            }
            else
            {
                data = trim_copy( strBuf.substr( pos + 1 ) );
            }
        }

        // cout << endl << data << endl;

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
// exifTimestamp example: 2012:09:30 15:38:49
// isoTimeStamp example:  2019-09-15T20:08:12
string MetaData::ConvertToLocalTimestamp( const string exifTimestamp )
{
    string isoTimestamp;
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
GC_STATUS MetaData::GetExifImageData( const string filepath, ExifFeatures &exifFeat )
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
            exifFeat.clear();
            retVal = GetExifData( filepath, "ImageWidth", data );
            if ( GC_OK == retVal )
            {
                exifFeat.imageDims.width = stoi( data );
                retVal = GetExifData( filepath, "ImageHeight", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.imageDims.height = stoi( data );
                }
                retVal = GetExifData( filepath, "DateTimeOriginal", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.captureTime = ConvertToLocalTimestamp( data );
                }
                retVal = GetExifData( filepath, "FNumber", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.fNumber = stod( data );
                }
                retVal = GetExifData( filepath, "ExposureTime", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.exposureTime = stod( data );
                }
                retVal = GetExifData( filepath, "ShutterSpeed", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.shutterSpeed = stod( data );
                }
                retVal = GetExifData( filepath, "ISO", data );
                if ( GC_OK == retVal )
                {
                    exifFeat.isoSpeedRating = stoi( data );
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[ExifMetadata::GetExifImageData] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// internal string trim methods
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string &trim( string &str )
{
   return ltrim( rtrim( str ) );
}
string &ltrim( string &str )
{
  auto it2 =  find_if( str.begin() , str.end() , [](char ch){ return !isspace<char>(ch , locale::classic() ) ; } );
  str.erase( str.begin() , it2);
  return str;
}
string &rtrim( string & str )
{
  auto it1 =  find_if( str.rbegin() , str.rend() , [](char ch){ return !isspace<char>(ch , locale::classic() ) ; } );
  str.erase( it1.base() , str.end() );
  return str;
}
string trim_copy( string const &str )
{
   auto s = str;
   return ltrim(rtrim(s));
}
