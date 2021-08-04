#include "log.h"
#include "calibexecutive.h"
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/diagnostic_information.hpp>

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = filesystem;
namespace pt = property_tree;

namespace gc
{

CalibExecutive::CalibExecutive() :
    calibType( NOT_SET )
{

}
GC_STATUS CalibExecutive::Calibrate( const std::string calibTargetImagePath, const std::string jsonParams, cv::Mat &imgResult )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        stringstream ss;
        ss << jsonParams;

        pt::ptree top_level;
        pt::json_parser::read_json( ss, top_level );

        string calibType = top_level.get< string >( "calibType", "" );
        string worldPtCSVFilepath = top_level.get< string >( "calibWorldPt_csv", "" );
        double stopSignFacetLength = top_level.get< double >( "stopSignFacetLength", -1.0 );
        string calibResultJsonFilepath = top_level.get< string >( "calibResult_json", "" );
        bool drawCalib = 1 == top_level.get< int >( "drawCalib", 0 );
        bool drawMoveSearchROIs = 1 == top_level.get< int >( "drawMoveSearchROIs", 0 );
        bool drawWaterLineSearchROI = 1 == top_level.get< int >( "drawWaterLineSearchROI", 0 );

        if ( "BowTie" == calibType )
        {
            retVal = CalibrateBowTie( calibTargetImagePath, worldPtCSVFilepath, calibResultJsonFilepath,
                                      imgResult, drawCalib, drawMoveSearchROIs, drawWaterLineSearchROI );
        }
        else if ( "StopSign" == calibType )
        {
            if ( 0.0 < stopSignFacetLength )
            {
                retVal = CalibrateStopSign( calibTargetImagePath, stopSignFacetLength,
                                            imgResult, drawCalib, drawMoveSearchROIs, drawWaterLineSearchROI );
            }
            else
            {

            }
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Invalid calibration type=" << calibType;
            retVal = GC_ERR;
        }

    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::ReadWorldCoordsFromCSVBowTie( const string csvFilepath, vector< vector< Point2d > > &worldCoords )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        ifstream file( csvFilepath );
        if ( !file.is_open() )
        {
            FILE_LOG( logERROR ) << "Could not open CSV filepath=" << csvFilepath;
            retVal = GC_ERR;
        }
        else
        {
            worldCoords.clear();

            string line;
            vector< string > vec;
            vector< Point2d > rowPts;

            getline( file, line );
            while ( getline( file, line ) )
            {
                rowPts.clear();
                algorithm::split( vec, line, is_any_of( "," ) );
                for ( size_t i = 0; i < vec.size(); i += 2 )
                {
                    rowPts.push_back( Point2d( atof( vec[ i ].c_str() ), atof( vec[ i + 1 ].c_str() ) ) );
                }
                worldCoords.push_back( rowPts );
            }
            file.close();
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::ReadWorldCoordsFromCSVBowTie] " << diagnostic_information( e );
        FILE_LOG( logERROR ) << "Could not read CSV filepath=" << csvFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateStopSign( const string imgFilepath, const double facetLength, cv::Mat &imgOut,
                                             const bool drawCalib, const bool drawMoveROIs, const bool drawMoveSearchROIs )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat searchImg = imread( imgFilepath, IMREAD_ANYCOLOR );
        if ( searchImg.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else if ( CV_8UC3 != searchImg.type() )

        FindSymbol findSym;
        double facetLength = 10.0;
        GC_STATUS retVal = stopSign.Calibrate( searchImg, facetLength );
        if ( GC_OK == retVal )
        {
            retVal = stopSign.DrawCalibration( searchImg, imgOut );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " facet length=" << facetLength;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateBowTie( const string imgFilepath, const string worldCoordsCsv, const string calibJson,
                                           Mat &imgOut, const bool drawCalib, const bool drawMoveROIs, const bool drawMoveSearchROIs )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat img = imread( imgFilepath, IMREAD_GRAYSCALE );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            vector< vector< Point2d > > worldCoords;
            retVal = ReadWorldCoordsFromCSVBowTie( worldCoordsCsv, worldCoords );
            if ( GC_OK == retVal )
            {
#ifdef DEBUG_BOWTIE_FIND
                retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE, DEBUG_FOLDER + "bowtie_find.png" );
#else
                retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE );
#endif
                if ( GC_OK == retVal )
                {
                    vector< vector< Point2d > > pixelCoords;
                    retVal = m_findCalibGrid.GetFoundPoints( pixelCoords );
                    if ( GC_OK == retVal )
                    {
                        if ( pixelCoords.size() != worldCoords.size() )
                        {
                            FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] Found pixel array row count does not equal world array count";
                            retVal = GC_ERR;
                        }
                        else
                        {
                            vector< Point2d > pixPtArray;
                            vector< Point2d > worldPtArray;
                            for ( size_t i = 0; i < pixelCoords.size(); ++i )
                            {
                                if ( pixelCoords[ i ].size() != worldCoords[ i ].size() )
                                {
                                    FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] Found pixel array column count does not equal world array count";
                                    retVal = GC_ERR;
                                    break;
                                }
                                for ( size_t j = 0; j < pixelCoords[ i ].size(); ++j )
                                {
                                    pixPtArray.push_back( pixelCoords[ i ][ j ] );
                                    worldPtArray.push_back( worldCoords[ i ][ j ] );
                                }
                            }
                            retVal = bowTie.Calibrate( pixPtArray, worldPtArray, Size( 2, 4 ), img.size(), img, imgOut, drawCalib, drawMoveROIs );
                            if ( GC_OK == retVal )
                            {
                                retVal = bowTie.Save( calibJson );
                            }
                        }
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " world coords csv=" << worldCoordsCsv;
        retVal = GC_EXCEPT;
    }

    return retVal;
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
