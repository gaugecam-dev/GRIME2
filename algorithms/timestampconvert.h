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

#ifndef TIMESTAMPCONVERT_H
#define TIMESTAMPCONVERT_H

#include "log.h"
#include "gc_types.h"
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace gc
{

class GcTimestamp
{
public:
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int dayOfYear;
};

class GcTimestampConvert
{
public:
    static GC_STATUS GetGcTimestampFromString( const std::string srcString, const int start_pos, const int tmStrlen, const std::string format, GcTimestamp &gcTime )
    {
        GC_STATUS retVal = GC_OK;
        try
        {
            size_t yy_pos = std::string::npos;
            size_t yyyy_pos = format.find( "yyyy" );
            if ( std::string::npos == yyyy_pos )
                yy_pos = format.find( "yy" );

            size_t mm_pos = format.find( "mm" );
            size_t dd_pos = format.find( "dd" );
            size_t HH_pos = format.find( "HH" );
            size_t MM_pos = format.find( "MM" );
            size_t SS_pos = format.find( "SS" );

            std::string srcStringAdj = srcString.substr( start_pos, tmStrlen );
            if ( std::string::npos != yyyy_pos )
            {
                gcTime.year = stoi( srcStringAdj.substr( yyyy_pos, 4 ) );
            }
            else if ( std::string::npos != yy_pos )
            {
                gcTime.year = 2000 + stoi( srcStringAdj.substr( yy_pos, 2 ) );
            }
            else
            {
                gcTime.year = 1955;
            }
            gcTime.month = std::string::npos == mm_pos ? 1 : stoi( srcStringAdj.substr( mm_pos, 2 ) );
            gcTime.day = std::string::npos == dd_pos ? 1 : stoi( srcStringAdj.substr( dd_pos, 2 ) );
            gcTime.hour = std::string::npos == HH_pos ? 0 : stoi( srcStringAdj.substr( HH_pos, 2 ) );
            gcTime.minute = std::string::npos == MM_pos ? 0 : stoi( srcStringAdj.substr( MM_pos, 2 ) );
            gcTime.second = std::string::npos == SS_pos ? 0 : stoi( srcStringAdj.substr( SS_pos, 2 ) );
            gcTime.dayOfYear = CalcDayOfYear( gcTime );
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::GetTimestampFromFilename] " << e.what();
            retVal = GC_EXCEPT;
        }

        return retVal;
    }
    static GC_STATUS ConvertDateToSeconds( const std::string srcString, const int start_pos, const int tmStrlen, const std::string format, long long &secsFromEpoch )
    {
        GcTimestamp gcTime;
        GC_STATUS retVal = GetGcTimestampFromString( srcString, start_pos, tmStrlen, format, gcTime );
        if ( GC_OK == retVal )
        {
            std::tm tm = {};
            std::stringstream ss( srcString );
            ss >> std::get_time( &tm, "%Y-%m-%d %H:%M:%S" );
            auto tmPt = std::chrono::system_clock::from_time_t(std::mktime(&tm));
            secsFromEpoch = std::chrono::time_point_cast< std::chrono::seconds >( tmPt ).time_since_epoch().count();
        }
        return retVal;
    }
    static GC_STATUS GetTimestampFromString( const std::string srcString, const int start_pos, const int tmStrlen, const std::string format, std::string &timestamp )
    {
        GcTimestamp gcTime;
        GC_STATUS retVal = GetGcTimestampFromString( srcString, start_pos, tmStrlen, format, gcTime );
        if ( GC_OK == retVal )
        {
            char buf[ 256 ];
            sprintf( buf, "%04d-%02d-%02dT%02d:%02d:%02d", gcTime.year, gcTime.month, gcTime.day, gcTime.hour, gcTime.minute, gcTime.second );
            timestamp = buf;
        }
        return retVal;
    }
    static std::string GetISOTimestampFromGcTimestamp( const GcTimestamp gcStamp )
    {
        char buf[ 256 ];
        sprintf( buf, "%04d-%02d-%02dT%02d:%02d:%02d", gcStamp.year, gcStamp.month, gcStamp.day, gcStamp.hour, gcStamp.minute, gcStamp.second );
        return std::string( buf );
    }
    static int CalcDayOfYear( const GcTimestamp gcStamp )
    {
        int doy = gcStamp.day;

        // check for leap year
        int days_in_feb = 28;
        if( (gcStamp.year % 4 == 0 && gcStamp.year % 100 != 0 ) || (gcStamp.year % 400 == 0) )
        {
            days_in_feb = 29;
        }

        switch(gcStamp.month)
        {
            case 2:
                doy += 31;
                break;
            case 3:
                doy += 31+days_in_feb;
                break;
            case 4:
                doy += 31+days_in_feb+31;
                break;
            case 5:
                doy += 31+days_in_feb+31+30;
                break;
            case 6:
                doy += 31+days_in_feb+31+30+31;
                break;
            case 7:
                doy += 31+days_in_feb+31+30+31+30;
                break;
            case 8:
                doy += 31+days_in_feb+31+30+31+30+31;
                break;
            case 9:
                doy += 31+days_in_feb+31+30+31+30+31+31;
                break;
            case 10:
                doy += 31+days_in_feb+31+30+31+30+31+31+30;
                break;
            case 11:
                doy += 31+days_in_feb+31+30+31+30+31+31+30+31;
                break;
            case 12:
                doy += 31+days_in_feb+31+30+31+30+31+31+30+31+30;
                break;
        }
        return doy;
    }

    static int DaysFromNumber( const int from_y, const int from_m, const int from_d,
                        const int to_y, const int to_m, const int to_d )
    {
        int returnVal = -1;
        try
        {
            boost::gregorian::date startDate( from_y, from_m, from_d );
            boost::gregorian::date endDate( to_y, to_m, to_d );
            boost::gregorian::days day_count = endDate - startDate;
            returnVal = day_count.days();
        }
        catch( boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::GetTimestampFromFilename] " << boost::diagnostic_information( e );
        }

        return returnVal;
    }

    static std::string DateFromDayNumber( const uint year, const uint month, const uint day, const uint days_to_add )
    {
        std::string retDateStr = "";
        try
        {
            boost::gregorian::days daysUntil( days_to_add );
            boost::gregorian::date startDate( year, month, day );
            boost::gregorian::date endDate = startDate + daysUntil;
            retDateStr = boost::gregorian::to_iso_extended_string( endDate );
            retDateStr[ 4 ] = ',';
            retDateStr[ 7 ] = ',';
        }
        catch( boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::GetTimestampFromFilename] " << boost::diagnostic_information( e );
        }
        return retDateStr;
    }
};

} // namespace gc

#endif // TIMESTAMPCONVERT_H
