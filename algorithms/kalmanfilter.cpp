#include "log.h"
#include "kalmanfilter.h"
#include "csvreader.h"
#include <vector>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "timestampconvert.h"

using namespace cv;
using namespace std;
using namespace boost;

namespace gc
{

Kalman::Kalman()
{

}

GC_STATUS Kalman::ApplyFromFile( const string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        ifstream inStream( jsonFilepath );
        if ( !inStream.is_open() )
        {
            FILE_LOG( logERROR ) << "[Kalman::ApplyFromFile] Could not open json parameters file: " << jsonFilepath;
            retVal = GC_ERR;
        }
        else
        {
            string jsonString( ( istreambuf_iterator< char >( inStream ) ),
                                 istreambuf_iterator< char >() );
            if ( jsonString.empty() )
            {
                FILE_LOG( logERROR ) << "[Kalman::ApplyFromFile] Json file held no parameters: " << jsonFilepath;
                retVal = GC_ERR;
            }
            else
            {
                retVal = ApplyFromString( jsonString );
            }
            inStream.close();
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Kalman::ApplyFromFile] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Kalman::ApplyFromString( const string jsonString )
{
    KalmanParams params;
    GC_STATUS retVal = ParamsFromJson( jsonString, params );
    if ( GC_OK == retVal )
    {
        retVal = Apply( params );
    }
    return retVal;
}
GC_STATUS Kalman::Apply( const KalmanParams params )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        CSVReader reader( params.inputCSVFilepath );
        vector< vector< string > > data = reader.getData();
        if ( data.empty() )
        {
            FILE_LOG( logERROR ) << "[Kalman::Apply] No data in input file " << params.inputCSVFilepath;
            retVal = GC_ERR;
        }
        else
        {
            ofstream outFile( params.outputCSVFilepath );
            if ( !outFile.is_open() )
            {
                FILE_LOG( logERROR ) << "[ApplyKalmanFilterToCSV] Could not open output file for writing " << params.outputCSVFilepath ;
                retVal = GC_ERR;
            }
            else
            {
                outFile << "Timestamp, measured, estimated" << endl;
                KalmanItem item;
                GC_STATUS retTimeGet = GcTimestampConvert::ConvertDateToSeconds( data[ 0 ][ params.datetimeColumn ],
                        params.timeStringStartCol, params.timeStringLength, params.datetimeFormat, item.secsSinceEpoch );
                if ( GC_OK == retTimeGet )
                {
                    item.measurement = stod( data[ 0 ][ params.measurementColumn ] );
                    item.prediction = item.measurement;
                    outFile << data[ 0 ][ params.datetimeColumn ] << "," << item.measurement << "," << item.prediction << endl;
                }

                KalmanFilter kf( 4, 2, 0 );
                kf.transitionMatrix = ( Mat_< float >( 4, 4 ) << 1,0,1,0,  0,1,0,1,  0,0,1,0,  0,0,0,1 );
                Mat_< float > measurement( 2, 1 ); measurement.setTo( Scalar( 0 ) );

                kf.statePre.at< float >( 0 ) = item.secsSinceEpoch;
                kf.statePre.at< float >( 1 ) = item.measurement;
                kf.statePre.at< float >( 2 ) = 0.0f;
                kf.statePre.at< float >( 3 ) = 0.0f;

                kf.statePost = kf.statePre;

                // setIdentity( kf.processNoiseCov, Scalar::all( 1e-4 ) );
                // setIdentity( kf.measurementNoiseCov, Scalar::all( 10 ) );
                // setIdentity( kf.errorCovPost, Scalar::all( 0.1 ) );

                setIdentity( kf.measurementMatrix );
                setIdentity( kf.processNoiseCov, Scalar::all( 1e-6 ) );
                setIdentity( kf.measurementNoiseCov, Scalar::all( 20 ) );
                setIdentity( kf.errorCovPost, Scalar::all( 1 ) );

                for ( size_t i = 1; i < data.size(); ++i )
                {
                    if ( GC_OK == retTimeGet )
                    {
                        retTimeGet = GcTimestampConvert::ConvertDateToSeconds( data[ i ][ params.datetimeColumn ],
                                params.timeStringStartCol, params.timeStringLength, params.datetimeFormat, item.secsSinceEpoch );
                        item.measurement = stod( data[ i ][ params.measurementColumn ] );
                        if ( -1.0 < item.measurement )
                        {
                            Mat prediction = kf.predict();
                            measurement( 0 ) = item.secsSinceEpoch;
                            measurement( 1 ) = item.measurement;

                            Mat estimated = kf.correct( measurement );
                            item.prediction = estimated.at< float >( 1 );
                            outFile << data[ i ][ params.datetimeColumn ] << "," << item.measurement << "," << item.prediction << endl;
                        }
                    }
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Kalman::Apply] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Kalman::ParamsToJsonFile( const KalmanParams params, string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        string jsonString;
        retVal = ParamsToJson( params, jsonString );
        if ( GC_OK == retVal )
        {
            ofstream outFile( jsonFilepath );
            if ( !outFile.is_open() )
            {
                FILE_LOG( logERROR ) << "[Kalman::ParamsToJsonFile] Could not open file to write: " << jsonFilepath;
                retVal = GC_ERR;
            }
            else
            {
                outFile << jsonString;
                outFile.close();
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Kalman::ParamsToJsonFile] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Kalman::ParamsFromJson( const string jsonString, KalmanParams &params )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        stringstream ss( jsonString );
        property_tree::ptree pt;
        property_tree::json_parser::read_json( ss, pt );
        params.datetimeFormat = pt.get< string >( "datetime_format" );
        params.outputCSVFilepath = pt.get< string >( "output_csv_filepath" );
        params.inputCSVFilepath = pt.get< string >( "input_csv_filepath" );
        params.firstDataRow = pt.get< int >( "first_data_row" );
        params.datetimeColumn = pt.get< int >( "datetime_column" );
        params.measurementColumn = pt.get< int >( "measurement_column" );
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Kalman::ParamsFromJson] " << boost::diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Kalman::ParamsToJson( const KalmanParams params, string &jsonString )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        stringstream ss;
        ss << "{" << endl;
        ss << "   \"datetime_format\":" << params.datetimeFormat << "," << endl;
        ss << "   \"output_csv_filepath\":" << params.outputCSVFilepath << "," << endl;
        ss << "   \"input_csv_filepath\":" << params.inputCSVFilepath << "," << endl;
        ss << "   \"first_data_row\":" << params.firstDataRow << "," << endl;
        ss << "   \"datetime_column\":" << params.datetimeColumn << "," << endl;
        ss << "   \"measurement_column\":" << params.measurementColumn << endl;
        ss << "}" << endl;
        jsonString = ss.str();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Kalman::ParamsToJson] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
