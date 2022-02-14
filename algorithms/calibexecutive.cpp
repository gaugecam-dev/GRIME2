#include "log.h"
#include "calibexecutive.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
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

static double Distance( const Point2d p1, const Point2d p2 )
{
    return sqrt( ( p2.x - p1.x ) * ( p2.x - p1.x ) +
                 ( p2.y - p1.y ) * ( p2.y - p1.y ) );
}

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
    out << "\"drawWaterLineSearchROI\": " << ( params.drawWaterLineSearchROI ? 1 : 0 ) << ", ";
    out << "\"targetRoi_x\": " << ( params.targetSearchROI.x ) << ", ";
    out << "\"targetRoi_y\": " << ( params.targetSearchROI.y ) << ", ";
    out << "\"targetRoi_width\": " << ( params.targetSearchROI.width ) << ", ";
    out << "\"targetRoi_height\": " << ( params.targetSearchROI.height ) << " }";
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
GC_STATUS CalibExecutive::Recalibrate( const Mat &img, const std::string calibType,
                                       double &rmseDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;

    string controlJson;
    if ( "StopSign" == calibType )
    {
        controlJson = stopSign.ControlJson();
    }
    else if ( "BowTie" == calibType )
    {
        controlJson = bowTie.ControlJson();
    }
    else
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Recalibrate] Invalid calibration type" ;
        retVal = GC_ERR;
    }

    if ( GC_OK == retVal )
    {
        retVal = Calibrate( img, controlJson, rmseDist, rmseX, rmseY );
    }

    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const Mat &img, const std::string jsonParams, cv::Mat &imgResult,
                                     double &rmseDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = Calibrate( img, jsonParams, rmseDist, rmseX, rmseY );
    if ( GC_OK == retVal )
    {
        retVal = DrawOverlay( img, imgResult );
    }
    return retVal;
}
GC_STATUS CalibExecutive::Calibrate( const cv::Mat &img, const std::string jsonParams,
                                     double &rmseDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        paramsCurrent.clear();

        stringstream ss;
        ss << jsonParams;
        // cout << ss.str() << endl;

        pt::ptree top_level;
        pt::json_parser::read_json( ss, top_level );

        paramsCurrent.calibType = top_level.get< string >( "calibType", "" );
        paramsCurrent.worldPtCSVFilepath = top_level.get< string >( "calibWorldPt_csv", "" );
        paramsCurrent.stopSignFacetLength = top_level.get< double >( "stopSignFacetLength", -1.0 );
        paramsCurrent.calibResultJsonFilepath = top_level.get< string >( "calibResult_json", "" );
        paramsCurrent.drawCalib = 1 == top_level.get< int >( "drawCalib", 0 );
        paramsCurrent.drawMoveSearchROIs = 1 == top_level.get< int >( "drawMoveSearchROIs", 0 );
        paramsCurrent.drawWaterLineSearchROI = 1 == top_level.get< int >( "drawWaterLineSearchROI", 0 );
        paramsCurrent.targetSearchROI.x = top_level.get< int >( "targetRoi_x", -1 );
        paramsCurrent.targetSearchROI.y = top_level.get< int >( "targetRoi_y", -1 );
        paramsCurrent.targetSearchROI.width = top_level.get< int >( "targetRoi_width", -1 );
        paramsCurrent.targetSearchROI.height = top_level.get< int >( "targetRoi_height", -1 );

        paramsCurrent.lineSearch_lftTop.x = top_level.get< int >( "searchPoly_lftTop_x", -1 );
        paramsCurrent.lineSearch_lftTop.y = top_level.get< int >( "searchPoly_lftTop_y", -1 );
        paramsCurrent.lineSearch_rgtTop.x = top_level.get< int >( "searchPoly_rgtTop_x", -1 );
        paramsCurrent.lineSearch_rgtTop.y = top_level.get< int >( "searchPoly_rgtTop_y", -1 );
        paramsCurrent.lineSearch_lftBot.x = top_level.get< int >( "searchPoly_lftBot_x", -1 );
        paramsCurrent.lineSearch_lftBot.y = top_level.get< int >( "searchPoly_lftBot_y", -1 );
        paramsCurrent.lineSearch_rgtBot.x = top_level.get< int >( "searchPoly_rgtBot_x", -1 );
        paramsCurrent.lineSearch_rgtBot.y = top_level.get< int >( "searchPoly_rgtBot_y", -1 );


        Mat imgFixed;
        if ( CV_8UC3 == img.type() )
        {
            cvtColor( img, imgFixed, cv::COLOR_BGR2GRAY );
        }
        else
        {
            imgFixed = img;
        }

        Rect searchBB;
        if ( "BowTie" == paramsCurrent.calibType )
        {
            retVal = CalibrateBowTie( imgFixed, jsonParams );
            if ( GC_OK == retVal )
            {
                retVal = bowTie.GetSearchRegionBoundingRect( searchBB );
            }
        }
        else if ( "StopSign" == paramsCurrent.calibType )
        {
            retVal = CalibrateStopSign( imgFixed, jsonParams );
            if ( GC_OK == retVal )
            {
                retVal = stopSign.GetSearchRegionBoundingRect( searchBB );
            }
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] Invalid calibration type=" <<
                                    ( paramsCurrent.calibType.empty() ? "empty()" : paramsCurrent.calibType );
            retVal = GC_ERR;
        }

        if ( GC_OK == retVal )
        {
            retVal = CalculateRMSE( imgFixed( searchBB ), rmseDist, rmseX, rmseY );
            if ( GC_OK != retVal )
            {
                rmseDist = rmseX = rmseY = -9999999.0;
                FILE_LOG( logWARNING ) << "[CalibBowtie::Calibrate] Could not calculate RMSE";
                retVal = GC_OK;
            }
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibExecutive::Calibrate] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut )
{
    GC_STATUS retVal = DrawOverlay( matIn, imgMatOut, paramsCurrent.drawMoveSearchROIs,
                                     paramsCurrent.drawMoveSearchROIs, paramsCurrent.drawWaterLineSearchROI );
    return retVal;
}
GC_STATUS CalibExecutive::DrawOverlay( const cv::Mat matIn, cv::Mat &imgMatOut,
                                       const bool drawCalib, const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = GC_OK;
    if ( "BowTie" == paramsCurrent.calibType )
    {
        CalibModelBowtie model = bowTie.GetModel();
        retVal = bowTie.DrawOverlay( matIn, imgMatOut, drawCalib, drawMoveROIs, drawSearchROI );
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = stopSign.DrawOverlay( matIn, imgMatOut, drawCalib, drawMoveROIs, drawSearchROI );
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
GC_STATUS CalibExecutive::CalibrateStopSign( const cv::Mat &img, const string &controlJson )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        clear();
        if ( CV_8UC3 != img.type() )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] A color image (RGB) is required for stop sign calibration";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point > searchLineCorners;
            searchLineCorners.push_back( paramsCurrent.lineSearch_lftTop );
            searchLineCorners.push_back( paramsCurrent.lineSearch_rgtTop );
            searchLineCorners.push_back( paramsCurrent.lineSearch_lftBot );
            searchLineCorners.push_back( paramsCurrent.lineSearch_rgtBot );
            retVal = stopSign.Calibrate( img, paramsCurrent.stopSignFacetLength, controlJson,
                                         searchLineCorners );
            if ( GC_OK == retVal )
            {
                retVal = stopSign.Save( paramsCurrent.calibResultJsonFilepath );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalibrateStopSign] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalibrateBowTie( const cv::Mat &img, const std::string &controlJson )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = findCalibGrid.InitBowtieTemplate( GC_BOWTIE_TEMPLATE_DIM, img.size() );
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
                Rect rect = ( -1 == paramsCurrent.targetSearchROI.x ||
                              -1 == paramsCurrent.targetSearchROI.y ||
                              -1 == paramsCurrent.targetSearchROI.width ||
                              -1 == paramsCurrent.targetSearchROI.height ) ? Rect( 0, 0, img.cols, img.rows ) : paramsCurrent.targetSearchROI;
                retVal = findCalibGrid.FindTargets( img, rect, MIN_BOWTIE_FIND_SCORE );
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
                            retVal = bowTie.Calibrate( pixPtArray, worldPtArray, controlJson, Size( 2, 4 ), img.size() );
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
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalibrateBowTie] " << e.what();
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
            // cout << endl << jsonString << endl;

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
cv::Rect &CalibExecutive::TargetRoi()
{
    if ( "BowTie" == paramsCurrent.calibType )
    {
        return bowTie.TargetRoi();
    }
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        return stopSign.TargetRoi();
    }
    FILE_LOG( logERROR ) << "[CalibExecutive::TargetRoi] No calibration type currently set";
    return nullRect;
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
    GC_STATUS retVal = GC_OK;

    if ( "BowTie" == paramsCurrent.calibType )
    {
        rectLeft = bowTie.MoveSearchROI( true );
        rectRight = bowTie.MoveSearchROI( true );
    }
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
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
    // TODO: StopSign placeholder (pull it back in)
#if 0
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = FindMoveTargetsStopSign( img, ptsFound );
    }
#endif
    else
    {
        FILE_LOG( logERROR ) << "[FindLine::FindMoveTargets] No valid calibration type currently set";
        retVal = GC_ERR;
    }

    return retVal;
}

// TODO: Fill in FindMoveTargetsStopSign()
// TODO: This method currently only handles translation and not rotation
#if 0
GC_STATUS CalibExecutive::FindMoveTargetsStopSign( const Mat &img, FindPointSet &ptsFound )
{
    GC_STATUS retVal = GC_OK;
    return retVal;
}
#endif

GC_STATUS CalibExecutive::FindMoveTargetsBowTie( const Mat &img, FindPointSet &ptsFound )
{
    GC_STATUS retVal = findCalibGrid.FindMoveTargetsBowTie( img, bowTie.TargetRoi(), ptsFound.lftPixel, ptsFound.rgtPixel );
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
    // TODO: StopSign placeholder (pull it back in)
#if 0
    else if ( "StopSign" == paramsCurrent.calibType )
    {
        retVal = MoveRefPointStopSign( lftRefPt, rgtRefPt );
    }
#endif
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
// TODO: Fill in MoveRefPointStopSign()
// TODO: This method currently only handles translation and not rotation
#if 0
GC_STATUS CalibExecutive::MoveRefPointStopSign( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = GC_OK;
    return retVal;
}
#endif

GC_STATUS CalibExecutive::CalculateRMSE( const cv::Mat &img, double &rmseEuclideanDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;
    rmseX = -9999999.0;
    rmseY = -9999999.0;
    rmseEuclideanDist = -9999999.0;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] The image must not be empty to calculate RMSE";
            retVal = GC_ERR;
        }
        else
        {

            vector< Point2d > reprojectedPts;
            Point2d projectedPt, reprojectedPt;
            for ( int row = 0; row < img.rows; ++row )
            {
                for ( int col = 0; col < img.cols; ++col )
                {
                    retVal = PixelToWorld( Point2d( col, row ), projectedPt );
                    if ( GC_OK == retVal )
                    {
                        retVal = WorldToPixel( projectedPt, reprojectedPt );
                        if ( GC_OK == retVal )
                        {
                            reprojectedPts.push_back( reprojectedPt );
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            if ( GC_OK == retVal )
            {
                int reprojectedIdx = 0;
                double diffSqrDist = 0.0;
                double diffSqrX = 0.0, diffSqrY = 0.0;
                for ( int row = 0; row < img.rows; ++row )
                {
                    for ( int col = 0; col < img.cols; ++col )
                    {
                        diffSqrX += pow( static_cast< double >( col ) - reprojectedPts[ reprojectedIdx ].x, 2 );
                        diffSqrY += pow( static_cast< double >( row ) - reprojectedPts[ reprojectedIdx ].y, 2 );
                        diffSqrDist += pow( Distance( Point2d( col, row ), reprojectedPts[ reprojectedIdx++ ] ), 2 );
                    }
                }

                rmseX = sqrt( diffSqrX );
                rmseY = sqrt( diffSqrY );
                rmseEuclideanDist = sqrt( diffSqrDist );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] " << e.what();
    }

    return retVal;
}
GC_STATUS CalibExecutive::CalculateRMSE( const std::vector< cv::Point2d > &foundPts, std::vector< cv::Point2d > &reprojectedPts,
                                         double &rmseEuclideanDist, double &rmseX, double &rmseY )
{
    GC_STATUS retVal = GC_OK;
    rmseX = -9999999.0;
    rmseY = -9999999.0;
    rmseEuclideanDist = -9999999.0;

    try
    {
        if ( 2 > foundPts.size() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] There must be more than one point to calculate RMSE";
            retVal = GC_ERR;
        }
        else
        {
            reprojectedPts.clear();
            Point2d projectedPt, reprojectedPt;
            for ( size_t i = 0; i < foundPts.size(); ++i )
            {
                retVal = PixelToWorld( foundPts[ i ], projectedPt );
                if ( GC_OK == retVal )
                {
                    retVal = WorldToPixel( projectedPt, reprojectedPt );
                    if ( GC_OK == retVal )
                    {
                        reprojectedPts.push_back( reprojectedPt );
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                vector< double > euclidDists;
                double diffSqrDist = 0.0;
                double diffSqrX = 0.0, diffSqrY = 0.0;
                for ( size_t i = 0; i < foundPts.size(); ++i )
                {
                    diffSqrX += pow( foundPts[ i ].x - reprojectedPts[ i ].x, 2 );
                    diffSqrY += pow( foundPts[ i ].y - reprojectedPts[ i ].y, 2 );
                    diffSqrDist += pow( Distance( foundPts[ i ], reprojectedPts[ i ] ), 2 );
                }

                rmseX = sqrt( diffSqrX );
                rmseY = sqrt( diffSqrY );
                rmseEuclideanDist = sqrt( diffSqrDist );
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::CalculateRMSE] " << e.what();
    }

    return retVal;
}


} // namespace gc
