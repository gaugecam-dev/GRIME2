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

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "gc_types.h"

namespace gc
{

class KalmanParams
{
public:

    std::string datetimeFormat;
    std::string outputCSVFilepath;
    std::string inputCSVFilepath;
    int firstDataRow;
    int measurementColumn;
    int datetimeColumn;
    int timeStringStartCol;
    int timeStringLength;
};

class KalmanItem
{
public:
    KalmanItem() :
        secsSinceEpoch( -1 ),
        measurement( -999.0 ),
        prediction( -999.0 )
    {}
    KalmanItem( const long long secondsSinceEpock, const double measure, const double predict ) :
        secsSinceEpoch( secondsSinceEpock ),
        measurement( measure ),
        prediction( predict )
    {}

    long long secsSinceEpoch;
    double measurement;
    double prediction;
};

class Kalman
{
public:
    Kalman();

    GC_STATUS ApplyFromFile( const std::string jsonFilepath );
    GC_STATUS ApplyFromString( const std::string jsonString );
    GC_STATUS Apply( const KalmanParams params );
    GC_STATUS ParamsToJsonFile( const KalmanParams params, std::string jsonFilepath );

private:
    GC_STATUS ParamsFromJson( const std::string jsonString, KalmanParams &params );
    GC_STATUS ParamsToJson( const KalmanParams params, std::string &jsonString );
};

} // namespace gc

#endif // KALMANFILTER_H
