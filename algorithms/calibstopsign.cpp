#include "log.h"
#include "calibstopsign.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <iterator>

#ifdef DEBUG_FIND_CALIB_SYMBOL
#undef DEBUG_FIND_CALIB_SYMBOL
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
static const std::string DEBUG_RESULT_FOLDER = "/var/tmp/water/";
#endif

static const double MIN_SYMBOL_CONTOUR_SIZE = 50;
static const double MIN_SYMBOL_CONTOUR_AREA = 1500;
static const int MIN_SYMBOL_CONTOUR_LENGTH = 7;
static const double MAX_SYMBOL_CONTOUR_ELONG = 1.5;
static const double MIN_SEARCH_LINE_LENGTH = 120.0;
using namespace cv;
using namespace std;
using namespace boost;

namespace gc
{

static double elongation( Moments m );

CalibStopSign::CalibStopSign()
{
#ifdef DEBUG_FIND_CALIB_SYMBOL
    if ( !filesystem::exists( DEBUG_RESULT_FOLDER ) )
    {
        bool bRet = filesystem::create_directories( DEBUG_RESULT_FOLDER );
        if ( !bRet )
        {
            FILE_LOG( logWARNING ) << "[CalibStopSign::CalibStopSign] Could not create debug folder " << DEBUG_RESULT_FOLDER;
        }
    }
#endif
}
void CalibStopSign::clear()
{
    matHomogPixToWorld = Mat();
    matHomogWorldToPix = Mat();
    model.clear();
}
// symbolPoints are clockwise ordered with 0 being the topmost left point
GC_STATUS CalibStopSign::Calibrate( const cv::Mat &img, const double octoSideLength, const std::string &controlJson,
                                    std::vector< Point > &searchLineCorners )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();
        std::vector< StopSignCandidate > candidates;

        cv::Mat1b mask;
        retVal = FindRed( img, mask, candidates );
        if ( GC_OK == retVal )
        {
            OctagonLines octoLines;
            Point2d ptTopLft, ptTopRgt, ptBotLft, ptBotRgt;
            for ( size_t i = 0; i < candidates.size(); ++i )
            {
                OctagonLines octoLines;
                retVal = FindCorners( mask, candidates[ i ].contour, octoLines );
                if ( GC_OK == retVal )
                {
                    vector< Point > corners;
                    retVal = FindDiagonals( mask, candidates[ i ].contour, octoLines );
                    if ( GC_OK == retVal )
                    {
                        retVal = CalcCorners( octoLines, model.pixelPoints );
                        if ( GC_OK == retVal )
                        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            Mat color;
                            img.copyTo( color );
                            for ( size_t i = 0; i < model.pixelPoints.size(); ++i )
                            {
                                line( color, Point( model.pixelPoints[ i ].x - 10, model.pixelPoints[ i ].y ),
                                             Point( model.pixelPoints[ i ].x + 10, model.pixelPoints[ i ].y ),
                                      Scalar( 0, 255, 255 ), 1 );
                                line( color, Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y - 10 ),
                                             Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y + 10 ),
                                      Scalar( 0, 255, 255 ), 1 );
                            }
                            imwrite( DEBUG_RESULT_FOLDER + "___FINAL.png", color );
#endif
                            retVal = CalcOctoWorldPoints( octoSideLength, model.worldPoints );
                            if ( GC_OK == retVal )
                            {
                                retVal = Calibrate( model.pixelPoints, model.worldPoints );
                                if ( GC_OK == retVal )
                                {
                                    retVal = CalcSearchLines( img, searchLineCorners, model.searchLines );
                                    if ( GC_OK == retVal )
                                    {
                                        retVal = CalcCenterAngle( model.worldPoints, model.center, model.angle );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if ( model.pixelPoints.empty() || model.worldPoints.empty() || model.searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] No valid calibration for drawing";
            retVal = GC_ERR;
        }
        else if ( matHomogPixToWorld.empty() || matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] System not calibrated";
            retVal = GC_ERR;
        }
        else
        {
            model.controlJson = controlJson;
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::CalcCenterAngle( const std::vector< cv::Point2d > &pts, cv::Point2d &center, double &angle )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( pts.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] Empty point sets";
            retVal = GC_ERR;
        }
        else
        {
            center = Point2d( 0.0, 0.0 );
            for ( size_t i = 0; i < pts.size(); ++i )
            {
                center.x += pts[ i ].x;
                center.y += pts[ i ].y;
            }
            center.x /= static_cast< double >( pts.size() );
            center.y /= static_cast< double >( pts.size() );

            vector< Point2d > ptsSortY = pts;
            sort( ptsSortY.begin(), ptsSortY.end(), []( Point2d const &a, Point const &b ) { return ( a.y < b.y ); } );
            Point2d ptLftTop = ptsSortY[ ptsSortY[ 0 ].x < ptsSortY[ 1 ].x ? 0 : 1 ];
            Point2d ptRgtTop = ptsSortY[ ptsSortY[ 0 ].x < ptsSortY[ 1 ].x ? 1 : 0 ];

            angle = atan2( ptRgtTop.y - ptLftTop.y, ptRgtTop.x - ptLftTop.x ) * ( 180.0 / CV_PI );
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::CalcCenterAngle] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::Calibrate(const std::vector< cv::Point2d > &pixelPts, const std::vector< cv::Point2d > &worldPts )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( pixelPts.empty() || worldPts.empty() || pixelPts.size() != worldPts.size() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] Invalid world and/or pixel point sets";
            retVal = GC_ERR;
        }
        else
        {
            matHomogPixToWorld = findHomography( pixelPts, worldPts );
            if ( matHomogPixToWorld.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] Could not find pixel to world coordinate homography";
                retVal = GC_ERR;
            }
            else
            {
                matHomogWorldToPix = findHomography( worldPts, pixelPts );
                if ( matHomogPixToWorld.empty() )
                {
                    FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] Could not find world to pixel coordinate homography";
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::Calibrate] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::GetLineEquation( const cv::Point2d pt1, const cv::Point2d pt2, double &slope, double &intercept )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        double deltaX = pt2.x - pt1.x;
        double deltaY = pt2.y - pt1.y;
        if ( numeric_limits< double >::epsilon() > deltaX )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::GetLineEquation] Invalid points: pt1 and pt2 cannot have the same value of X";
            retVal = GC_ERR;
        }
        else
        {
            slope = deltaY / deltaX;
            intercept = pt2.y - slope * pt2.x;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::GetLineEquation] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::GetSearchRegionBoundingRect( cv::Rect &rect )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( model.searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::GetSearchRegionBoundingRect] System not calibrated";
            retVal = GC_ERR;
        }
        else
        {
            int left = std::min( model.searchLines[ 0 ].top.x, model.searchLines[ 0 ].bot.x );
            int top = std::min( model.searchLines[ 0 ].top.y, model.searchLines[ model.searchLines.size() - 1 ].top.y );
            int right = std::max( model.searchLines[ model.searchLines.size() - 1 ].top.x, model.searchLines[ model.searchLines.size() - 1 ].bot.x );
            int bottom = std::max( model.searchLines[ 0 ].bot.y, model.searchLines[ model.searchLines.size() - 1 ].bot.y );
            rect = Rect( left, top, right - left, bottom - top );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::GetSearchRegionBoundingRect] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibStopSign::CalcSearchLines( const Mat &img, vector< Point > &searchLineCorners, std::vector< LineEnds > &searchLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalcSearchLines] Invalid input image";
            retVal = GC_ERR;
        }
        else if ( 4 != searchLineCorners.size() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalcSearchLines] Invalid search line corner point count. Must be 4";
            retVal = GC_ERR;
        }
        else
        {
            sort( searchLineCorners.begin(), searchLineCorners.end(), []( Point const &a, Point const &b ) { return ( a.y < b.y ); } );

            const Point lftTop = searchLineCorners[ searchLineCorners[ 0 ].x > searchLineCorners[ 1 ].x ? 1 : 0 ];
            const Point rgtTop = searchLineCorners[ searchLineCorners[ 0 ].x > searchLineCorners[ 1 ].x ? 0 : 1 ];
            const Point lftBot = searchLineCorners[ searchLineCorners[ 2 ].x > searchLineCorners[ 3 ].x ? 3 : 2 ];
            const Point rgtBot = searchLineCorners[ searchLineCorners[ 2 ].x > searchLineCorners[ 3 ].x ? 2 : 3 ];

            searchLines.clear();
            searchLines.push_back( LineEnds( lftTop, lftBot ) );

            int widthTop = rgtTop.x - lftTop.x;
            int widthBot = rgtBot.x - lftBot.x;
            int width = std::max( widthTop, widthBot );
            double topInc = static_cast< double >( widthTop ) / static_cast< double >( width );
            double botInc = static_cast< double >( widthBot ) / static_cast< double >( width );

            double slopeTop, interceptTop;
            retVal = GetLineEquation( lftTop, rgtTop, slopeTop, interceptTop );
            if ( GC_OK == retVal )
            {
                double slopeBot, interceptBot;
                retVal = GetLineEquation( lftBot, rgtBot, slopeBot, interceptBot );
                if ( GC_OK == retVal )
                {
                    double topY, botY;
                    double topX = static_cast< double >( lftTop.x );
                    double botX = static_cast< double >( lftBot.x );

                    for ( int i = 0; i < width; ++i )
                    {
                        topX += topInc;
                        botX += botInc;
                        topY = slopeTop * topX + interceptTop;
                        botY = slopeBot * botX + interceptBot;
                        searchLines.push_back( LineEnds( Point2d( topX, topY ), Point2d( botX, botY ) ) );
                    }
                    searchLines.push_back( LineEnds( rgtTop, rgtBot ) );
                }
            }
        }

        return retVal;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::CalcSearchLines] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::Load( const std::string jsonCalFilepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( jsonCalFilepath.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Load] Bow tie calibration string is empty";
            retVal = GC_ERR;
        }
        else
        {
            stringstream ss;
            ss << jsonCalFilepath;
            // cout << endl << endl << ss.str() << endl;
            property_tree::ptree ptreeTop;
            property_tree::json_parser::read_json( ss, ptreeTop );

            model.imgSize.width = ptreeTop.get< int >( "imageWidth", 0 );
            model.imgSize.height = ptreeTop.get< int >( "imageHeight", 0 );
            property_tree::ptree ptreeCalib = ptreeTop.get_child( "PixelToWorld" );

            Point2d ptTemp;
            model.pixelPoints.clear();
            model.worldPoints.clear();

            BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeCalib.get_child( "points" ) )
            {
                ptTemp.x = node.second.get< double >( "pixelX", 0.0 );
                ptTemp.y = node.second.get< double >( "pixelY", 0.0 );
                model.pixelPoints.push_back( ptTemp );
                ptTemp.x = node.second.get< double >( "worldX", 0.0 );
                ptTemp.y = node.second.get< double >( "worldY", 0.0 );
                model.worldPoints.push_back( ptTemp );
            }

            const property_tree::ptree &ptreeMoveSearch = ptreeTop.get_child( "TargetSearchRegion" );
            model.wholeTargetRegion.x =      ptreeMoveSearch.get< int >( "x", 0 );
            model.wholeTargetRegion.y =      ptreeMoveSearch.get< int >( "y", 0 );
            model.wholeTargetRegion.width =  ptreeMoveSearch.get< int >( "width", 0 );
            model.wholeTargetRegion.height = ptreeMoveSearch.get< int >( "height", 0 );

            Point ptTop, ptBot;
            model.searchLines.clear();
            BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeTop.get_child( "SearchLines" ) )
            {
                ptTop.x = node.second.get< int >( "topX", std::numeric_limits< int >::min() );
                ptTop.y = node.second.get< int >( "topY", std::numeric_limits< int >::min() );
                ptBot.x = node.second.get< int >( "botX", std::numeric_limits< int >::min() );
                ptBot.y = node.second.get< int >( "botY", std::numeric_limits< int >::min() );
                model.searchLines.push_back( LineEnds( ptTop, ptBot ) );
            }

#ifdef LOG_CALIB_VALUES
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Camera calibration association points";
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Columns=" << cols << " Rows=" << rows;
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
#endif
            if ( 5 > model.pixelPoints.size() )
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::Load] Invalid association point count";
                retVal = GC_ERR;
            }
            else
            {
#ifdef LOG_CALIB_VALUES
                for ( size_t i = 0; i < m_settings.pixelPoints.size(); ++i )
                {
                    FILE_LOG( logINFO ) << "[r=" << i / cols << " c=" << i % cols << "] " << \
                                           " pixelX=" << m_settings.pixelPoints[ i ].x << " pixelY=" << m_settings.pixelPoints[ i ].y << \
                                           " worldX=" << m_settings.worldPoints[ i ].x << " worldY=" << m_settings.worldPoints[ i ].y;
                }
#endif

                model.controlJson = ptreeTop.get< string >( "control_json", "{}" );

                Mat matIn, matOut;
                retVal = Calibrate( model.pixelPoints, model.worldPoints );
            }
#ifdef LOG_CALIB_VALUES
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            FILE_LOG( logINFO ) << "Search lines";
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            for ( size_t i = 0; i < m_settings.searchLines.size(); ++i )
            {
                FILE_LOG( logINFO ) << "[index=" << i << "] " << \
                                       " topX=" << m_settings.searchLines[ i ].top.x << " topY=" << m_settings.searchLines[ i ].top.y << \
                                       " botX=" << m_settings.searchLines[ i ].bot.x << " botY=" << m_settings.searchLines[ i ].bot.y;
            }
#endif
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::Save( const std::string jsonCalFilepath )
{
    GC_STATUS retVal = GC_OK;

    if ( model.pixelPoints.empty() || model.worldPoints.empty() ||
         model.pixelPoints.size() != model.worldPoints.size() || model.searchLines.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::Save] Empty cal point vector(s). Saves not possible without a calibrated object";
        retVal = GC_ERR;
    }
    else if ( jsonCalFilepath.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::Save] Calibration filepath is empty";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            ofstream fileStream( jsonCalFilepath );
            if ( fileStream.is_open() )
            {
                fileStream << "{" << endl;
                fileStream << "  \"calibType\":\"StopSign\"" << "," << endl;
                fileStream << "  \"imageWidth\":" << model.imgSize.width << "," << endl;
                fileStream << "  \"imageHeight\":" << model.imgSize.height << "," << endl;
                fileStream << "  \"PixelToWorld\": " << endl;
                fileStream << "  {" << endl;
                fileStream << "    \"points\": [" << endl;
                fileStream << fixed << setprecision( 3 );
                for ( size_t i = 0; i < model.pixelPoints.size() - 1; ++i )
                {
                    fileStream << "      { \"pixelX\": " << model.pixelPoints[ i ].x << ", " << \
                                          "\"pixelY\": " << model.pixelPoints[ i ].y << ", " << \
                                          "\"worldX\": " << model.worldPoints[ i ].x << ", " << \
                                          "\"worldY\": " << model.worldPoints[ i ].y << " }," << endl;
                }
                fileStream << "      { \"pixelX\": " << model.pixelPoints[ model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"pixelY\": " << model.pixelPoints[ model.pixelPoints.size() - 1 ].y << ", " << \
                                      "\"worldX\": " << model.worldPoints[ model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"worldY\": " << model.worldPoints[ model.pixelPoints.size() - 1 ].y << " }" << endl;
                fileStream << "    ]" << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"TargetSearchRegion\": " << endl;
                fileStream << "  {" << endl;
                fileStream << fixed << setprecision( 0 );
                fileStream << "      \"x\": " <<      model.wholeTargetRegion.x << ", " << \
                                    "\"y\": " <<      model.wholeTargetRegion.y << ", " << \
                                    "\"width\": " <<  model.wholeTargetRegion.width << ", " << \
                                    "\"height\": " << model.wholeTargetRegion.height << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"SearchLines\": [" << endl;
                for ( size_t i = 0; i < model.searchLines.size() - 1; ++i )
                {
                    fileStream << "      { \"topX\": " << model.searchLines[ i ].top.x << ", " << \
                                          "\"topY\": " << model.searchLines[ i ].top.y << ", " << \
                                          "\"botX\": " << model.searchLines[ i ].bot.x << ", " << \
                                          "\"botY\": " << model.searchLines[ i ].bot.y << " }," << endl;
                }
                fileStream << "      { \"topX\": " << model.searchLines[ model.searchLines.size() - 1 ].top.x << ", " << \
                                      "\"topY\": " << model.searchLines[ model.searchLines.size() - 1 ].top.y << ", " << \
                                      "\"botX\": " << model.searchLines[ model.searchLines.size() - 1 ].bot.x << ", " << \
                                      "\"botY\": " << model.searchLines[ model.searchLines.size() - 1 ].bot.y << " }" << endl;
                fileStream << "  ]," << endl;

                string escaped;
                for ( size_t i = 0; i < model.controlJson.size(); ++i )
                {
                    if ( model.controlJson[ i ] == '\"' )
                        escaped += "\\\"";
                    else
                        escaped += model.controlJson[ i ];
                }

                fileStream << "  \"control_json\": \"" << escaped << "\"" << endl;
                fileStream << "}" << endl;
                fileStream.close();
            }
            else
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::Save]"
                                        "Could not open calibration save file " << jsonCalFilepath;
                retVal = GC_ERR;
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::Save] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS CalibStopSign::CalcOctoWorldPoints( const double sideLength, std::vector< cv::Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        pts.clear();
        double cornerLength = sideLength * sqrt( 2.0 );
        double top = cornerLength * 2.0 + sideLength;
        double row2 = cornerLength + sideLength;
        double row3 = cornerLength;
        double bot = 0.0;
        double lft = 0.0;
        double col2 = cornerLength;
        double col3 = cornerLength + sideLength;
        double rgt = cornerLength * 2.0 + sideLength;
        pts.push_back( Point2d( col2, top ) );
        pts.push_back( Point2d( col3, top ) );
        pts.push_back( Point2d( rgt, row2 ) );
        pts.push_back( Point2d( rgt, row3 ) );
        pts.push_back( Point2d( col3, bot ) );
        pts.push_back( Point2d( col2, bot ) );
        pts.push_back( Point2d( lft, row3 ) );
        pts.push_back( Point2d( lft, row2 ) );
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::CalcWorldPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::FindRed( const cv::Mat &img, cv::Mat1b &redMask, std::vector< StopSignCandidate > &symbolCandidates )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::FindRed] Cannot find red in an empty image";
            retVal = GC_ERR;
        }
        else if ( img.type() != CV_8UC3 )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::FindRed] Image must be an 8-bit BGR image to find red";
            retVal = GC_ERR;
        }
        else
        {
            Mat3b hsv;
            cvtColor( img, hsv, COLOR_BGR2HSV );

            Mat1b mask1, mask2;
            inRange( hsv, Scalar( 0, 70, 50 ), Scalar( 10, 255, 255 ), mask1 );
            inRange( hsv, Scalar( 170, 70, 50 ), Scalar( 180, 255, 255 ), mask2 );

            redMask = mask1 | mask2;

            vector< vector< Point > > contours;
            findContours( redMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            img.copyTo( color );
            imwrite( DEBUG_RESULT_FOLDER + "red_mask.png", redMask );
#endif

            Moments m;
            double area, elong;
            symbolCandidates.clear();
            for ( size_t i = 0; i < contours.size(); ++i )
            {
                if ( MIN_SYMBOL_CONTOUR_LENGTH <= contours[ i ].size() )
                {
                    area = contourArea( contours[ i ] );
                    if ( MIN_SYMBOL_CONTOUR_AREA <= area )
                    {
                        m = moments( contours[ i ] );
                        elong = elongation( m );
#ifdef DEBUG_FIND_CALIB_SYMBOL
                        cout << "elongation=" << elong << endl;
#endif
                        if ( MAX_SYMBOL_CONTOUR_ELONG >= elong )
                        {
                            symbolCandidates.push_back( StopSignCandidate( contours[ i ], area, elong ) );
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            drawContours( color, contours, i, Scalar( 0, 255, 255 ), 3 );
#endif
                        }
                    }
                }
            }
            if ( symbolCandidates.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::FindRed] No symbol candidates found";
                retVal = GC_ERR;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            else
            {
                imwrite( DEBUG_RESULT_FOLDER + "candidates.png", color );
            }
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::FindRed] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::FindCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( contour.size() < MIN_SYMBOL_CONTOUR_SIZE )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalibStopSignCorners] Contour must have at least " << MIN_SYMBOL_CONTOUR_SIZE << " contour points";
            retVal = GC_ERR;
        }
        else if ( mask.empty() || CV_8UC1 != mask.type() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalibStopSignCorners] Invalid mask image";
            retVal = GC_ERR;
        }
        else
        {
            Mat edges = Mat::zeros( mask.size(), CV_8UC1 );
            drawContours( edges, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), 1 );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            imwrite( DEBUG_RESULT_FOLDER + "candidate_contour.png", edges );
#endif
            Rect bb = boundingRect( contour );
            int swathSize = bb.height / 5;
            RotatedRect rotRect = fitEllipse( contour );
            Mat scratch = Mat::zeros( mask.size(), CV_8UC1 );
            line( scratch, rotRect.center, Point( 0, rotRect.center.y ), Scalar( 255 ), swathSize );
            scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "left_edge_pts_swath.png", scratch );
#endif

            int top = rotRect.center.y - swathSize / 2;
            top = 0 > top ? 0 : top;
            int bot = rotRect.center.y + swathSize / 2;
            bot = scratch.rows <= bot ? scratch.rows - 1 : bot;

            Rect rect( 0, top, rotRect.center.x, bot - top );
            Point2d lftPt1, lftPt2;
            retVal = GetLineEndPoints( scratch, rect, lftPt1, lftPt2 );
            if ( GC_OK == retVal )
            {
                scratch = 0;
                line( scratch, rotRect.center, Point( scratch.cols - 1, rotRect.center.y ), Scalar( 255 ), swathSize );
                scratch &= edges;

                rect = Rect( rotRect.center.x, top, scratch.cols - rotRect.center.x, bot - top );
                Point2d rgtPt1, rgtPt2;
                retVal = GetLineEndPoints( scratch, rect, rgtPt1, rgtPt2 );
                if ( GC_OK == retVal )
                {
                    scratch = 0;
                    line( scratch, rotRect.center, Point( rotRect.center.x, 0 ), Scalar( 255 ), swathSize );
                    scratch &= edges;

                    int lft = rotRect.center.x - swathSize / 2;
                    lft = 0 > lft ? 0 : lft;
                    int rgt = rotRect.center.x + swathSize / 2;
                    rgt = scratch.cols <= rgt ? scratch.cols - 1 : rgt;

                    rect = Rect( lft, 0, rgt - lft, rotRect.center.y );
                    Point2d topPt1, topPt2;
                    retVal = GetLineEndPoints( scratch, rect, topPt1, topPt2 );
                    if ( GC_OK == retVal )
                    {
                        scratch = 0;
                        line( scratch, rotRect.center, Point( rotRect.center.x, scratch.rows - 1 ), Scalar( 255 ), swathSize );
                        scratch &= edges;

                        rect = Rect( lft, rotRect.center.y, rgt - lft, scratch.rows - rotRect.center.y );
                        Point2d botPt1, botPt2;
                        retVal = GetLineEndPoints( scratch, rect, botPt1, botPt2 );
                        if ( GC_OK == retVal )
                        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            line( color, lftPt1, lftPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, rgtPt1, rgtPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, topPt1, topPt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, botPt1, botPt2, Scalar( 0, 0, 255 ), 1 );
#endif
                            retVal = LineIntersection( StopSignLine( topPt1, topPt2 ), StopSignLine( lftPt1, lftPt2 ), octoLines.top.pt1 );
                            if ( GC_OK == retVal )
                            {
                                octoLines.left.pt2 = octoLines.top.pt1;
                                retVal = LineIntersection( StopSignLine( topPt1, topPt2 ), StopSignLine( rgtPt1, rgtPt2 ), octoLines.top.pt2 );
                                if ( GC_OK == retVal )
                                {
                                    octoLines.right.pt1 = octoLines.top.pt2;
                                    retVal = LineIntersection( StopSignLine( botPt1, botPt2 ), StopSignLine( lftPt1, lftPt2 ), octoLines.bot.pt2 );
                                    if ( GC_OK == retVal )
                                    {
                                        octoLines.left.pt1 = octoLines.bot.pt2;
                                        retVal = LineIntersection( StopSignLine( botPt1, botPt2 ), StopSignLine( rgtPt1, rgtPt2 ), octoLines.right.pt2 );
                                        if ( GC_OK == retVal )
                                        {
                                            octoLines.bot.pt1 = octoLines.right.pt2;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "symbol_edges.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::CalibStopSignCorners] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::CalcCorners( const OctagonLines octoLines, std::vector< cv::Point2d > &symbolCorners )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Point2d pt;
        symbolCorners.clear();
        retVal = LineIntersection( octoLines.topLeft, octoLines.top, pt );
        if ( GC_OK == retVal )
        {
            symbolCorners.push_back( pt );
            retVal = LineIntersection( octoLines.top, octoLines.topRight, pt );
            if ( GC_OK == retVal )
            {
                symbolCorners.push_back( pt );
                retVal = LineIntersection( octoLines.topRight, octoLines.right, pt );
                if ( GC_OK == retVal )
                {
                    symbolCorners.push_back( pt );
                    retVal = LineIntersection( octoLines.right, octoLines.botRight, pt );
                    if ( GC_OK == retVal )
                    {
                        symbolCorners.push_back( pt );
                        retVal = LineIntersection( octoLines.botRight, octoLines.bot, pt );
                        if ( GC_OK == retVal )
                        {
                            symbolCorners.push_back( pt );
                            retVal = LineIntersection( octoLines.bot, octoLines.botLeft, pt );
                            if ( GC_OK == retVal )
                            {
                                symbolCorners.push_back( pt );
                                retVal = LineIntersection( octoLines.botLeft, octoLines.left, pt );
                                if ( GC_OK == retVal )
                                {
                                    symbolCorners.push_back( pt );
                                    retVal = LineIntersection( octoLines.left, octoLines.topLeft, pt );
                                    if ( GC_OK == retVal )
                                    {
                                        symbolCorners.push_back( pt );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::CalcCorners] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
GC_STATUS CalibStopSign::LineIntersection( const StopSignLine line1, const StopSignLine line2, Point2d &r )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d x = line2.pt1 - line1.pt1;
        Point2d d1 = line1.pt2 - line1.pt1;
        Point2d d2 = line2.pt2 - line2.pt1;

        double cross = d1.x * d2.y - d1.y * d2.x;
        if (abs(cross) < numeric_limits< double >::epsilon() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::LineIntersection] Lines are parallel";
            return GC_ERR;
        }

        double t1 = ( x.x * d2.y - x.y * d2.x ) / cross;
        r = line1.pt1 + d1 * t1;
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::LineIntersection] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::FindDiagonals( const cv::Mat &mask, const std::vector< cv::Point > &contour, OctagonLines &octoLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( contour.size() < MIN_SYMBOL_CONTOUR_SIZE )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalibStopSignCorners] Contour must have at least " << MIN_SYMBOL_CONTOUR_SIZE << " contour points";
            retVal = GC_ERR;
        }
        else if ( mask.empty() || CV_8UC1 != mask.type() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalibStopSignCorners] Invalid mask image";
            retVal = GC_ERR;
        }
        else
        {
            Mat edges = Mat::zeros( mask.size(), CV_8UC1 );
            drawContours( edges, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), 1 );
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            imwrite( DEBUG_RESULT_FOLDER + "candidate_contour.png", edges );
#endif
            Rect bb = boundingRect( contour );
            int swathSize = bb.height / 5;
            RotatedRect rotRect = fitEllipse( contour );
            Mat scratch = Mat::zeros( mask.size(), CV_8UC1 );
            line( scratch, rotRect.center, octoLines.top.pt1, Scalar( 255 ), swathSize );
            scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "top_left_edge_pts_swath.png", scratch );
#endif

            Rect rect( octoLines.top.pt1.x, octoLines.top.pt1.y, rotRect.center.x - octoLines.top.pt1.x,
                       rotRect.center.y - octoLines.top.pt1.y );

            retVal = GetLineEndPoints( scratch, rect, octoLines.topLeft.pt1, octoLines.topLeft.pt2 );
            if ( GC_OK == retVal )
            {
                scratch = 0;
                line( scratch, rotRect.center, octoLines.top.pt2, Scalar( 255 ), swathSize );
                scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                imwrite( DEBUG_RESULT_FOLDER + "top_right_edge_pts_swath.png", scratch );
#endif

                rect = Rect( rotRect.center.x, octoLines.top.pt2.y, octoLines.top.pt2.x - rotRect.center.x,
                             rotRect.center.y - octoLines.top.pt2.y );

                retVal = GetLineEndPoints( scratch, rect, octoLines.topRight.pt1, octoLines.topRight.pt2 );
                if ( GC_OK == retVal )
                {
                    scratch = 0;
                    line( scratch, rotRect.center, octoLines.bot.pt2, Scalar( 255 ), swathSize );
                    scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                    imwrite( DEBUG_RESULT_FOLDER + "bot_left_edge_pts_swath.png", scratch );
#endif

                    rect = Rect( octoLines.bot.pt2.x, rotRect.center.y, rotRect.center.x - octoLines.bot.pt2.x,
                                 octoLines.bot.pt2.y - rotRect.center.y );
                    retVal = GetLineEndPoints( scratch, rect, octoLines.botLeft.pt1, octoLines.botLeft.pt2 );
                    if ( GC_OK == retVal )
                    {
                        scratch = 0;
                        line( scratch, rotRect.center, octoLines.bot.pt1, Scalar( 255 ), swathSize );
                        scratch &= edges;
#ifdef DEBUG_FIND_CALIB_SYMBOL
                        imwrite( DEBUG_RESULT_FOLDER + "bot_right_edge_pts_swath.png", scratch );
#endif

                        rect = Rect( rotRect.center.x, rotRect.center.y, octoLines.bot.pt1.x - rotRect.center.x,
                                     octoLines.bot.pt1.y - rotRect.center.y );

                        retVal = GetLineEndPoints( scratch, rect, octoLines.botRight.pt1, octoLines.botRight.pt2 );
                        if ( GC_OK == retVal )
                        {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            line( color, octoLines.topLeft.pt1, octoLines.topLeft.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.topRight.pt1, octoLines.topRight.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.botLeft.pt1, octoLines.botLeft.pt2, Scalar( 0, 0, 255 ), 1 );
                            line( color, octoLines.botRight.pt1, octoLines.botRight.pt2, Scalar( 0, 0, 255 ), 1 );
#endif
                        }
                    }
                }
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            imwrite( DEBUG_RESULT_FOLDER + "symbol_edges_diagonal.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::FindDiagonals] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::GetLineEndPoints( cv::Mat &mask, const cv::Rect rect, cv::Point2d &pt1, cv::Point2d &pt2 )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat search = mask( rect );
#ifdef DEBUG_FIND_CALIB_SYMBOL
        imwrite( DEBUG_RESULT_FOLDER + "pt_search_img.png", mask );
        imwrite( DEBUG_RESULT_FOLDER + "pt_search_rect.png", search );
#endif

        vector< Point > pts;
        retVal = GetNonZeroPoints( search, pts );
        if ( GC_OK == retVal )
        {
            for ( size_t i = 0; i < pts.size(); ++i )
            {
                pts[ i ].x += rect.x;
                pts[ i ].y += rect.y;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            cvtColor( mask, color, COLOR_GRAY2BGR );
            drawContours( color, vector< vector< Point > >( 1, pts ), -1, Scalar( 0, 255, 255 ), 1 );
            imwrite( DEBUG_RESULT_FOLDER + "pt_search_pts.png", color );
#endif

            Vec4d lne;
            fitLine( pts, lne, DIST_L12, 0.0, 0.01, 0.01 );

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // line equation: y = mx + b
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            Point2d pt1x0, pt1y0, pt2x0, pt2y0;

            double a = lne[ 1 ];
            double b = -lne[ 0 ];
            double c = lne[ 0 ] * lne[ 3 ] - lne[ 1 ] * lne[ 2 ];

            double denom = ( 0.0 == a ? std::numeric_limits< double >::epsilon() : a );
            pt1y0.y = 0.0;
            pt1y0.x = c / -denom;
            pt2y0.y = static_cast< double >( mask.rows - 1 );
            pt2y0.x = ( b * pt2y0.y + c ) / -denom;

            denom = ( 0.0 == b ? std::numeric_limits< double >::epsilon() : b );
            pt1x0.x = 0.0;
            pt1x0.y = c / -denom;
            pt2x0.x = static_cast< double >( mask.cols - 1 );
            pt2x0.y = ( a * pt2x0.x + c ) / -denom;

            if ( 0.0 <= pt1y0.x && 0.0 <= pt1y0.y && mask.cols > pt1y0.x && mask.rows > pt1y0.y )
            {
                pt1 = pt1y0;
            }
            else
            {
                pt1 = pt1x0;
            }


            if ( 0.0 <= pt2y0.x && 0.0 <= pt2y0.y && mask.cols > pt2y0.x && mask.rows > pt2y0.y )
            {
                pt2 = pt2y0;
            }
            else
            {
                pt2 = pt2x0;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            line( color, pt1, pt2, Scalar( 0, 255, 0 ), 1 );
            imwrite( DEBUG_RESULT_FOLDER + "pt_search_line.png", color );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::GetLineEndPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;

}
GC_STATUS CalibStopSign::GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::GetNonZeroPoints] Can not get points from an empty image";
            retVal = GC_ERR;
        }
        else
        {
            pts.clear();
            uchar *pPix;
            for( int r = 0; r < img.rows; ++r )
            {
                pPix = img.ptr< uchar >( r );
                for ( int c = 0; c < img.cols; ++c )
                {
                    if ( 0 != pPix[ c ] )
                        pts.push_back( Point( c, r ) );
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::GetNonZeroPoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::PixelToWorld( const Point2d ptPixel, Point2d &ptWorld )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogPixToWorld.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::PixelToWorld] No calibration for pixel to world conversion";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptPixel );
            perspectiveTransform( vecIn, vecOut, matHomogPixToWorld );
            ptWorld = vecOut[ 0 ];
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::PixelToWorld] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::WorldToPixel( const Point2d ptWorld, Point2d &ptPixel )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::WorldToPixel]"
                                    "No calibration for world to pixel conversion";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptWorld );
            perspectiveTransform( vecIn, vecOut, matHomogWorldToPix );
            ptPixel = vecOut[ 0 ];
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::WorldToPixel] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibStopSign::DrawOverlay( const cv::Mat &img, cv::Mat &result, const bool drawCalib,
                                      const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( matHomogPixToWorld.empty() || matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::DrawCalibration] System not calibrated";
            retVal = GC_ERR;
        }
        else if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::DrawCalibration] Empty image";
            retVal = GC_ERR;
        }
        else
        {
            if ( CV_8UC1 == img.type() )
            {
                cvtColor( img, result, COLOR_GRAY2BGR );
            }
            else if ( CV_8UC3 == img.type() )
            {
                img.copyTo( result );
            }
            else
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::DrawCalibration] Invalid image type";
                retVal = GC_ERR;
            }

            if ( model.pixelPoints.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibStopSign::DrawCalibration] No symbol points to draw";
                retVal = GC_ERR;
            }
            else if ( GC_OK == retVal )
            {
                double dim = static_cast< double >( std::max( result.cols, result.rows ) );
                int lineWidth = max( 1, cvRound( dim / 900.0 ) );
                int targetRadius = lineWidth * 5;

                if ( drawCalib )
                {
                    line( result, Point( model.pixelPoints[ 0 ].x - targetRadius, model.pixelPoints[ 0 ].y ),
                                  Point( model.pixelPoints[ 0 ].x + targetRadius, model.pixelPoints[ 0 ].y ), Scalar( 0, 255, 0 ), lineWidth );
                    line( result, Point( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y - targetRadius ),
                                  Point( model.pixelPoints[ 0 ].x, model.pixelPoints[ 0 ].y + targetRadius ), Scalar( 0, 255, 0 ), lineWidth );
                    circle( result, model.pixelPoints[ 0 ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                    for ( size_t i = 1; i < model.pixelPoints.size(); ++i )
                    {
                        line( result, model.pixelPoints[ i - 1 ], model.pixelPoints[ i ], Scalar( 255, 0, 0 ), lineWidth );
                        line( result, Point( model.pixelPoints[ i ].x - targetRadius, model.pixelPoints[ i ].y ),
                                      Point( model.pixelPoints[ i ].x + targetRadius, model.pixelPoints[ i ].y ), Scalar( 0, 255, 0 ), lineWidth );
                        line( result, Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y - targetRadius ),
                                      Point( model.pixelPoints[ i ].x, model.pixelPoints[ i ].y + targetRadius ), Scalar( 0, 255, 0 ), lineWidth );
                        circle( result, model.pixelPoints[ i ], targetRadius, Scalar( 0, 255, 0 ), lineWidth );
                    }
                    line( result, model.pixelPoints[ 0 ], model.pixelPoints[ model.pixelPoints.size() - 1 ], Scalar( 255, 0, 0 ), lineWidth );

                    Point2d ptLftTopPix( 0.0, 0.0 );
                    Point2d ptRgtTopPix( static_cast< double >( result.cols - 1 ), 0.0 );
                    Point2d ptLftBotPix( 0.0, static_cast< double >( result.rows - 1 ) );
                    Point2d ptRgtBotPix( static_cast< double >( result.cols - 1 ), static_cast< double >( result.rows - 1 ) );

                    Point2d ptLftTopW, ptRgtTopW, ptLftBotW, ptRgtBotW;
                    retVal = PixelToWorld( ptLftTopPix, ptLftTopW );
                    if ( GC_OK == retVal )
                    {
                        retVal = PixelToWorld( ptRgtTopPix, ptRgtTopW );
                        if ( GC_OK == retVal )
                        {
                            retVal = PixelToWorld( ptLftBotPix, ptLftBotW );
                            if ( GC_OK == retVal )
                            {
                                retVal = PixelToWorld( ptRgtBotPix, ptRgtBotW );
                                if ( GC_OK == retVal )
                                {
                                    double minXW = std::min( ptLftTopW.x, ptLftBotW.x );
                                    double maxXW = std::max( ptRgtTopW.x, ptRgtBotW.x );
                                    double minYW = std::min( ptLftTopW.y, ptRgtTopW.y );
                                    double maxYW = std::max( ptLftBotW.y, ptRgtBotW.y );
                                    if ( maxYW < minYW )
                                    {
                                        swap( minYW, maxYW );
                                    }

                                    double incX = ( maxXW - minXW ) / 10.0;
                                    double incY = ( maxYW - minYW ) / 10.0;

                                    bool isFirst;
                                    char msg[ 256 ];
                                    Point2d pt1, pt2;
                                    for ( double r = minYW; r < maxYW; r += incY )
                                    {
                                        isFirst = true;
                                        for ( double c = minXW; c < maxXW; c += incX )
                                        {
                                            retVal = WorldToPixel( Point2d( c, r ), pt1 );
                                            if ( GC_OK == retVal )
                                            {
                                                if ( isFirst )
                                                {
                                                    isFirst = false;
                                                    sprintf( msg, "%.1f cm", r );
                                                    putText( result, msg, Point( 10, pt1.y - 10 ), FONT_HERSHEY_PLAIN, static_cast< double >( lineWidth ) / 1.5, Scalar( 0, 0, 255 ), lineWidth );
                                                }
                                                retVal = WorldToPixel( Point2d( c + incX, r ), pt2 );
                                                if ( GC_OK == retVal )
                                                {
                                                    line( result, pt1, pt2, Scalar( 0, 255, 255 ), lineWidth );
                                                    retVal = WorldToPixel( Point2d( c, r + incY ), pt2 );
                                                    if ( GC_OK == retVal )
                                                    {
                                                        line( result, pt1, pt2, Scalar( 0, 255, 255 ), lineWidth );
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if ( drawMoveROIs )
                {
                    // rectangle( result, model.wholeTargetRegion,  );
                }
                if ( drawSearchROI )
                {
                    if ( model.searchLines.empty() )
                    {
                        putText( result, "NO SEARCH REGION SET", Point( 50, result.cols - 100 ), FONT_HERSHEY_PLAIN, 2.7, Scalar( 0, 0, 255 ), 3 );
                        rectangle( result, Rect( 100, 100, result.cols - 200, result.rows - 200 ), Scalar( 0, 0, 255 ), 3 );
                        line( result, Point( 100, 100 ), Point( result.cols - 200, result.rows - 200 ), Scalar( 0, 0, 255 ), 3 );
                        line( result, Point( 100, result.rows - 200 ), Point( result.cols - 200, 100 ), Scalar( 0, 0, 255 ), 3 );
                    }
                    else
                    {
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibStopSign::DrawCalibration] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

// helper functions
static double elongation( Moments m )
{
    double x = m.mu20 + m.mu02;
    double y = 4 * m.mu11 * m.mu11 + ( m.mu20 - m.mu02 ) * ( m.mu20 - m.mu02 );
    double srt = sqrt( y );
    if ( x - srt > DBL_EPSILON )
        return ( x + srt ) / ( x - srt );
    else
        return 1.0;
}

} // namespace gc