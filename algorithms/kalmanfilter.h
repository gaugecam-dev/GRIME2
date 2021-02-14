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
