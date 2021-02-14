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
#include "ransacstreamflow.h"
#include <random>
#include <iostream>
#include <fstream>
#include <string>
#include "csvreader.h"
#include "timestampconvert.h"

using namespace std;

namespace gc
{

RansacStreamflow::RansacStreamflow()
{

}

bool is_numeric (const std::string& str)
{
    std::istringstream ss(str);
    double dbl;
    ss >> dbl;      // try to read the number
    ss >> std::ws;  // eat whitespace after number

    if (!ss.fail() && ss.eof()) {
        return true;  // is-a-number
    } else {
        return false; // not-a-number
    }
}
std::string remove_whitespace( const std::string &str )
{
    std::string s = str;
    s.erase( std::remove_if( s.begin(), s.end(), ::isspace), s.end() );
    return str;
}

GC_STATUS RansacStreamflow::CreateRandomStreamflowModel( const std::string filepathCSV, const std::string filepathResult,
                                                         const std::string timestampFormat, const int timestampCol,
                                                         const int valueCol /*. const int chances */ )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        CSVReader reader( filepathCSV );
        vector< vector< string > > data = reader.getData();
        if ( data.empty() )
        {
            FILE_LOG( logERROR ) << "[VisAppFeats::ReadCSV] No data in file " << filepathCSV;
            retVal = GC_ERR;
        }
        else
        {
            // outer is day of year inner is discharge value
            vector< vector< double > > discharges, d2015, d2016, d2017;
            for ( size_t i = 0; i < 366; ++i )
            {
                discharges.push_back( vector< double >() );
                d2015.push_back( vector< double >() );
                d2016.push_back( vector< double >() );
                d2017.push_back( vector< double >() );
            }

            GcTimestamp gcTimeStamp;
            string strScratch;
            for ( size_t i = 33; i < data.size(); ++i )
            {
                retVal = GcTimestampConvert::GetGcTimestampFromString( data[ i ][ timestampCol ], 0, 10, timestampFormat, gcTimeStamp );
                if ( GC_OK == retVal )
                {
                    if ( !data[ i ][ valueCol ].empty() )
                    {
                        strScratch = remove_whitespace( data[ i ][ valueCol ] );
                        if ( is_numeric( strScratch ) )
                        {
                            if ( 2014 > gcTimeStamp.year || ( 2014 == gcTimeStamp.year  && 9 >= gcTimeStamp.month ) )
                                discharges[ gcTimeStamp.dayOfYear - 1 ].push_back( stod( strScratch ) );
                            else if ( ( 2014 == gcTimeStamp.year && 10 <= gcTimeStamp.month ) ||
                                      ( 2015 == gcTimeStamp.year && 9 >= gcTimeStamp.month ) )
                            {
                                d2015[ gcTimeStamp.dayOfYear - 1 ].push_back( stod( strScratch ) );
                            }
                            else if ( ( 2015 == gcTimeStamp.year && 10 <= gcTimeStamp.month ) ||
                                      ( 2016 == gcTimeStamp.year && 9 >= gcTimeStamp.month ) )
                            {
                                d2016[ gcTimeStamp.dayOfYear - 1 ].push_back( stod( strScratch ) );
                            }
                            else if ( ( 2016 == gcTimeStamp.year && 10 <= gcTimeStamp.month ) ||
                                      ( 2017 == gcTimeStamp.year && 9 >= gcTimeStamp.month ) )
                            {
                                d2017[ gcTimeStamp.dayOfYear - 1 ].push_back( stod( strScratch ) );
                            }
                        }
                    }
                }
            }
            std::random_device rd;  //Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

            double sum, sum15, sum16, sum17;
            ofstream outFile( filepathResult );
            outFile << "Day of year, mean, median, 2015 mean, 2015 median, 2016 mean, 2016 median, 2017 mean, 2017 median" << endl;
            vector< double > items, items15, items16, items17;
#if 1
            for ( size_t j = 0; j < discharges.size(); ++j )
            {
                sum = sum15 = sum16 = sum17 = 0.0;
                items.clear();
                items15.clear();
                items16.clear();
                items17.clear();
                for ( size_t i = 0; i < discharges[ j ].size(); ++i )
                {
                    sum += discharges[ j ][ i ];
                    items.push_back( discharges[ j ][ i ] );
                }
                sort( items.begin(), items.end() );
                outFile << j << "," << 0.02832 * ( sum / static_cast< double >( discharges[ j ].size() ) ) << "," << 0.02832 * items[ discharges[ j ].size() >> 1 ] << ",";

                for ( size_t i = 0; i < d2015[ j ].size(); ++i )
                {
                    sum15 += d2015[ j ][ i ];
                    items15.push_back( d2015[ j ][ i ] );
                }
                sort( items15.begin(), items15.end() );
                outFile << 0.02832 * ( sum15 / static_cast< double >( d2015[ j ].size() ) ) << "," << 0.02832 * items15[ d2015[ j ].size() >> 1 ] << ",";

                for ( size_t i = 0; i < d2016[ j ].size(); ++i )
                {
                    sum17 += d2016[ j ][ i ];
                    items17.push_back( d2016[ j ][ i ] );
                    if ( items16.size() - 1 >= j )
                    {
                        sum16 += d2016[ j ][ i ];
                        items16.push_back( d2016[ j ][ i ] );
                    }
                }
                if ( items16.size() - 1 >= j )
                {
                    sort( items16.begin(), items16.end() );
                    outFile << 0.02832 * ( sum16 / static_cast< double >( d2016[ j ].size() ) ) << "," << 0.02832 * items16[ d2016[ j ].size() >> 1 ] << ",";
                }
                else
                {
                    outFile << "0.0,0.0,";
                }

                for ( size_t i = 0; i < d2017[ j ].size(); ++i )
                {
                    sum17 += d2017[ j ][ i ];
                    items17.push_back( d2017[ j ][ i ] );
                }
                sort( items17.begin(), items17.end() );
                outFile << 0.02832 * ( sum17 / static_cast< double >( d2017[ j ].size() ) ) << "," << 0.02832 * items17[ d2017[ j ].size() >> 1 ] << endl;
            }
#else
            int index;
            for ( size_t j = 0; j < discharges.size(); ++j )
            {
                sum = 0.0;
                items.clear();
                std::uniform_int_distribution<> distrib( 0, discharges[ j ].size() );
                for ( int i = 0; i < chances; ++i )
                {
                    index = distrib( gen );
                    sum += discharges[ j ][ index ];
                    items.push_back( discharges[ j ][ index ] );
                }
                sort( items.begin(), items.end() );
                outFile << j << "," << 0.02832 * ( sum / static_cast< double >( chances ) ) << "," << 0.02832 * items[ chances >> 1 ] << endl;
            }
#endif
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[RansacStreamflow::CreateRandomStreamflowModel] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
