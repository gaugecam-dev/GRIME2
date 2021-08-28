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

ostream &operator<<( ostream &out, CalibExecParams &params )
{

    out << "{ \"calibType\": \"" << params.calibType << "\", ";
    out << "\"calibWorldPt_csv\": \"" << params.worldPtCSVFilepath << "\", ";
    out << "\"stopSignFacetLength\": " << params.stopSignFacetLength << ", ";
    out << "\"calibResult_json\": \"" << params.calibResultJsonFilepath << "\", ";
    out << "\"drawCalib\": " << ( params.drawCalib ? 1 : 0 ) << ", ";
    out << "\"drawMoveSearchROIs\": " << ( params.drawMoveSearchROIs ? 1 : 0 ) << ", ";
    out << "\"drawWaterLineSearchROI\": " << ( params.drawWaterLineSearchROI ? 1 : 0 ) << " }";
    return out;
}

CalibExecutive::CalibExecutive()
{

}
void CalibExecutive::clear()
{
    bowTie.clear();
    stopSign.clear();
}
GC_STATUS CalibExecutive::Calibrate( const std::string calibTargetImagePath, const std::string jsonParams, cv::Mat &imgResult )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        paramsCurrent.clear();

        stringstream ss;
        ss << jsonParams;
        cout << ss.str() << endl;

        pt::ptree top_level;
        pt::json_parser::read_json( ss, top_level );

        paramsCurrent.calibType = top_level.get< string >( "calibType", "" );
        paramsCurrent.worldPtCSVFilepath = top_level.get< string >( "calibWorldPt_csv", "" );
        paramsCurrent.stopSignFacetLength = top_level.get< double >( "stopSignFacetLength", -1.0 );
        paramsCurrent.calibResultJsonFilepath = top_level.get< string >( "calibResult_json", "" );
        paramsCurrent.drawCalib = 1 == top_level.get< int >( "drawCalib", 0 );
        paramsCurrent.drawMoveSearchROIs = 1 == top_level.get< int >( "drawMoveSearchROIs", 0 );
        paramsCurrent.drawWaterLineSearchROI = 1 == top_level.get< int >( "drawWaterLineSearchROI", 0 );

        if ( "BowTie" == paramsCurrent.calibType )
        {
            retVal = CalibrateBowTie( calibTargetImagePath, imgResult );
        }
        else if ( "StopSign" == paramsCurrent.calibType )
        {
            retVal = CalibrateStopSign( calibTargetImagePath );
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Invalid calibration type=" <<
                                    ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
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
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                                       const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        CalibModel model = bowTie.GetModel();
        retVal = bowTie.Calibrate( model.pixelPoints, model.worldPoints, model.gridSize, matIn.size(),
                                   matIn, imgMatOut, drawCalib, drawMoveROIs, drawSearchROI );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.DrawCalibration( matIn, imgMatOut, drawCalib, drawMoveROIs, drawSearchROI );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::DrawOverlay] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = bowTie.PixelToWorld( pixelPt, worldPt );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.PixelToWorld( pixelPt, worldPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::PixelToWorld] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
    }
    return retVal;
}
GC_STATUS CalibExecutive::WorldToPixel( const cv::Point2d worldPt, cv::Point2d &pixelPt )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = bowTie.WorldToPixel( worldPt, pixelPt );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.WorldToPixel( worldPt, pixelPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::WorldToPixel] Invalid calibration type=" << ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
        retVal = GC_ERR;
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
GC_STATUS CalibExecutive::CalibrateStopSign( const string imgFilepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        clear();
        Mat searchImg = imread( imgFilepath, IMREAD_ANYCOLOR );
        if ( searchImg.empty() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] Could not open image file " << imgFilepath;
            retVal = GC_ERR;
        }
        else if ( CV_8UC3 != searchImg.type() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] A color image (RGB) is required for stop sign calibration";
            retVal = GC_ERR;
        }
        else
        {
            retVal = stopSign.Calibrate( searchImg, paramsCurrent.stopSignFacetLength );
            if ( GC_OK == retVal )
            {
                retVal = stopSign.Save( paramsCurrent.calibResultJsonFilepath );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] " << e.what();
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " facet length=" << paramsCurrent.stopSignFacetLength;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateBowTie( const string imgFilepath, Mat &imgOut )
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
            retVal = findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, Size( GC_IMAGE_SIZE_WIDTH, GC_IMAGE_SIZE_HEIGHT ) );
            if ( GC_OK != retVal )
            {
                FILE_LOG( logERROR ) << "[VisApp::VisApp] Could not initialize bowtie templates for calibration";
            }
            else
            {
                vector< vector< Point2d > > worldCoords;
                retVal = ReadWorldCoordsFromCSVBowTie( paramsCurrent.worldPtCSVFilepath, worldCoords );
                if ( GC_OK == retVal )
                {
#ifdef DEBUG_BOWTIE_FIND
                    retVal = m_findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE, DEBUG_FOLDER + "bowtie_find.png" );
#else
                    retVal = findCalibGrid.FindTargets( img, MIN_BOWTIE_FIND_SCORE );
#endif
                    if ( GC_OK == retVal )
                    {
                        vector< vector< Point2d > > pixelCoords;
                        retVal = findCalibGrid.GetFoundPoints( pixelCoords );
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
                                retVal = bowTie.Calibrate( pixPtArray, worldPtArray, Size( 2, 4 ), img.size(), img, imgOut,
                                                           paramsCurrent.drawCalib, paramsCurrent.drawMoveSearchROIs, paramsCurrent.drawMoveSearchROIs );
                                if ( GC_OK == retVal )
                                {
                                    retVal = bowTie.Save( paramsCurrent.calibResultJsonFilepath );
                                }
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
        FILE_LOG( logERROR ) << "Image=" << imgFilepath << " world coords csv=" << paramsCurrent.worldPtCSVFilepath;
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::Load( const string jsonFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();
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
            if ( calibTypeString == "BowTie" )
            {
                stopSign.clear();
                paramsCurrent.calibType = "BowTie";
                retVal = findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, Size( GC_IMAGE_SIZE_WIDTH, GC_IMAGE_SIZE_HEIGHT ) );
                if ( GC_OK == retVal )
                {
                    retVal = bowTie.Load( ss.str() );
                    if ( GC_OK == retVal )
                    {
                        Mat scratch( bowTie.GetModel().imgSize, CV_8UC1 );
                        retVal = findCalibGrid.SetMoveTargetROI( scratch, bowTie.MoveSearchROI( true ), bowTie.MoveSearchROI( false ) );
                    }
                }
            }
            else if ( calibTypeString == "StopSign" )
            {
                bowTie.clear();
                findCalibGrid.clear();
                paramsCurrent.calibType = "StopSign";
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
std::vector< LineEnds > &CalibExecutive::SearchLines()
{
    if ( "BowTie" == paramsCurrent.calibType )
    {
        return bowTie.SearchLines();
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        return stopSign.SearchLines();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::SearchLines] No calibration type currently set";
    return nullSearchLines;
}
GC_STATUS CalibExecutive::GetMoveSearchROIs( Rect &rectLeft , Rect &rectRight )
{
    // TODO: Not yet implemented
    return GC_ERR;
}
GC_STATUS CalibExecutive::SetMoveSearchROIs( const cv::Mat img, const cv::Rect rectLeft, const cv::Rect rectRight )
{
    return findCalibGrid.SetMoveTargetROI( img, rectLeft, rectRight );
}
GC_STATUS CalibExecutive::FindMoveTargets( const Mat &img, FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;

    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = FindMoveTargetsBowTie( img, ptsFound );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = FindMoveTargetsStopSign( img, ptsFound );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
// TODO: Fill in FindMoveTargetsStopSign()
GC_STATUS CalibExecutive::FindMoveTargetsStopSign( const Mat &img, FindPointSet &ptsFound )
{
    // TODO: This method currently only handles translation and not rotation
    GC_STATUS retVal = GC_OK;
    return retVal;
}
GC_STATUS CalibExecutive::FindMoveTargetsBowTie( const Mat &img, FindPointSet &ptsFound )
{
    // TODO: This method currently only handles translation and not rotation
    GC_STATUS retVal = findCalibGrid.FindMoveTargetsBowTie( img, ptsFound.lftPixel, ptsFound.rgtPixel );
    if ( GC_OK == retVal )
    {
        ptsFound.ctrPixel.x = ( ptsFound.lftPixel.x + ptsFound.rgtPixel.x ) / 2.0;
        ptsFound.ctrPixel.y = ( ptsFound.lftPixel.y + ptsFound.rgtPixel.y ) / 2.0;
    }
    return retVal;
}
GC_STATUS CalibExecutive::MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = GC_OK;

    if ( "BowTie" == paramsCurrent.calibType )
    {
        retVal = MoveRefPointBowTie( lftRefPt, rgtRefPt );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = MoveRefPointStopSign( lftRefPt, rgtRefPt );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}
GC_STATUS CalibExecutive::MoveRefPointBowTie( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = bowTie.MoveRefPoint( lftRefPt, rgtRefPt );
    return retVal;
}
GC_STATUS CalibExecutive::MoveRefPointStopSign( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    // TODO: This method currently only handles translation and not rotation
    GC_STATUS retVal = GC_OK;
    return retVal;
}

} // namespace gc
