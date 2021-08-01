#include "log.h"
#include "calibexecutive.h"
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;

namespace gc
{

CalibExecutive::CalibExecutive() :
    calibType( NOT_SET )
{

}
GC_STATUS CalibExecutive::Calibrate( const std::string calibTargetImagePath, const std::string jsonParams )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat img = imread( calibTargetImagePath );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Could not read calib target image: " << calibTargetImagePath;
            retVal = GC_ERR;
        }
        else
        {
            stringstream ss;
            ss << jsonParams;

            property_tree::ptree pt;
            property_tree::read_json( ss, pt );

            string calibTypeString = pt.get< string >( "calibType", "NotSet" );
            if ( string::npos != calibTypeString.find( "BowTie" ) )
            {
                calibType = BOWTIE;
                retVal = bowTie.Load( ss.str() );
            }
            else if ( string::npos != calibTypeString.find( "StopSign" ) )
            {
                calibType = STOPSIGN;
                retVal = stopSign.Load( ss.str() );

            }
            else if ( "NotSet" == calibTypeString )
            {
                FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] No calibration type specified in calibration file";
                retVal = GC_ERR;
            }

        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const cv::Mat &calibTargetImage, const std::string jsonParams )
{
    return GC_OK;
}
GC_STATUS CalibExecutive::Load( const string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !fs::exists( jsonFilepath ) )
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << jsonFilepath << " does not exist";
            retVal = GC_ERR;
        }
        else
        {
            std::string jsonString;
            fs::load_string_file( jsonFilepath, jsonString );

            stringstream ss;
            ss << jsonString;

            property_tree::ptree pt;
            property_tree::read_json( ss, pt );

            string calibTypeString = pt.get< string >( "calibType", "NotSet" );
            if ( string::npos != calibTypeString.find( "BowTie" ) )
            {
                calibType = BOWTIE;
                retVal = bowTie.Load( ss.str() );
            }
            else if ( string::npos != calibTypeString.find( "StopSign" ) )
            {
                calibType = STOPSIGN;
                retVal = stopSign.Load( ss.str() );

            }
            else if ( "NotSet" == calibTypeString )
            {
                FILE_LOG( logERROR ) << "[CalibExecutive::Load] No calibration type specified in calibration file";
                retVal = GC_ERR;
            }
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
