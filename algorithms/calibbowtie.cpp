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
#include "calibbowtie.h"
#include "findline.h"
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/format.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#ifdef __DEBUG_CALIB
#undef __DEBUG_CALIB
#endif

#ifdef LOG_CALIB_VALUES
#undef LOG_CALIB_VALUES
#endif

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

namespace gc
{

enum PIX_POS_INDEX
{
    PIX_POS_X = 0,
    PIX_POS_Y = 1
};

void CalibBowtie::clear()
{
    m_model.clear();
    m_matHomogPixToWorld = Mat();
    m_matHomogWorldToPix = Mat();
    // m_worldToPixParams.clear();
}
GC_STATUS CalibBowtie::Calibrate( const vector< Point2d > pixelPts, const vector< Point2d > worldPts, const std::string &controlJson,
                                  const Size gridSize, const Size imgSize )
{
    GC_STATUS retVal = GC_OK;
    if ( pixelPts.size() != worldPts.size() || pixelPts.empty() || worldPts.empty() ||
         gridSize.width * gridSize.height != static_cast< int >( pixelPts.size() ) || 0 >= gridSize.width )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::Calibrate] Calibration world/pixel coordinate point counts do not match or are empty";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            m_model.clear();
            m_model.imgSize = imgSize;
            m_model.gridSize = gridSize;
            m_model.pixelPoints.clear();
            m_model.worldPoints.clear();
            for ( size_t i = 0; i < pixelPts.size(); ++i )
            {
                m_model.pixelPoints.push_back( pixelPts[ i ] );
                m_model.worldPoints.push_back( worldPts[ i ] );
            }

            m_matHomogPixToWorld = findHomography( m_model.pixelPoints, m_model.worldPoints );
            invert( m_matHomogPixToWorld, m_matHomogWorldToPix );

            // Mat homogWorldToPix;
            // invert( m_matHomogPixToWorld, homogWorldToPix );
            // m_worldToPixParams.clear();
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 0, 0 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 0, 1 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 0, 2 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 1, 0 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 1, 1 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 1, 2 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 2, 0 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 2, 1 ) );
            // m_worldToPixParams.push_back( homogWorldToPix.at< double >( 2, 2 ) );

            retVal = CalcSearchSwaths();
            if ( GC_OK == retVal )
            {
                m_model.moveSearchRegionLft = Rect( std::max( 0, cvRound( m_model.pixelPoints[ 0 ].x ) - GC_BOWTIE_TEMPLATE_DIM ),
                                                    std::max( 0, cvRound( m_model.pixelPoints[ 0 ].y )- GC_BOWTIE_TEMPLATE_DIM ),
                                                    std::min( imgSize.width - cvRound( m_model.pixelPoints[ 0 ].x ), GC_BOWTIE_TEMPLATE_DIM * 2 ),
                                                    std::min( imgSize.height - cvRound( m_model.pixelPoints[ 0 ].y ), GC_BOWTIE_TEMPLATE_DIM * 2 ) );
                size_t idx = static_cast< size_t >( m_model.gridSize.width - 1 );
                m_model.moveSearchRegionRgt = Rect( std::max( 0, cvRound( m_model.pixelPoints[ idx ].x ) - GC_BOWTIE_TEMPLATE_DIM ),
                                                    std::max( 0, cvRound( m_model.pixelPoints[ idx ].y )- GC_BOWTIE_TEMPLATE_DIM ),
                                                    std::min( imgSize.width - cvRound( m_model.pixelPoints[ idx ].x ), GC_BOWTIE_TEMPLATE_DIM * 2 ),
                                                    std::min( imgSize.height - cvRound( m_model.pixelPoints[ idx ].y ), GC_BOWTIE_TEMPLATE_DIM * 2 ) );

                int width = std::max( cvRound( m_model.pixelPoints[ idx ].x - m_model.pixelPoints[ 0 ].x ),
                                      cvRound( m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].x - m_model.pixelPoints[ m_model.pixelPoints.size() - 2 ].x ) );
                int height = std::max( cvRound( m_model.pixelPoints[ m_model.pixelPoints.size() - 2 ].y - m_model.pixelPoints[ 0 ].y ),
                                       cvRound( m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].y - m_model.pixelPoints[ idx ].y ) );
                int left = std::max( 0, cvRound( m_model.pixelPoints[ 0 ].x ) - width / 2 );
                int top = std::max( 0, cvRound( m_model.pixelPoints[ 0 ].y ) - height / 2 );

                width *= 2;
                height *= 2;
                if ( left + width > imgSize.width - 1 )
                {
                    width = imgSize.width - left - 1;
                }
                if ( top + height > imgSize.height - 1 )
                {
                    height = imgSize.height - top - 1;
                }

                m_model.wholeTargetRegion = Rect( left, top, width, height );
            }
            if ( m_model.pixelPoints.empty() || m_model.worldPoints.empty() || m_model.searchLines.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibBowtie::Calibrate] No valid calibration for drawing";
                retVal = GC_ERR;
            }
            else if ( m_matHomogPixToWorld.empty() || m_matHomogWorldToPix.empty() )
            {
                FILE_LOG( logERROR ) << "[CalibBowtie::Calibrate] System not calibrated";
                retVal = GC_ERR;
            }
            else
            {
                m_model.controlJson = controlJson;
            }
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::Calibrate] " << e.what();
            return GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS CalibBowtie::GetSearchRegionBoundingRect( cv::Rect &rect )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( m_model.searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::GetSearchRegionBoundingRect] System not calibrated";
            retVal = GC_ERR;
        }
        else
        {
            int left = std::min( m_model.searchLines[ 0 ].top.x, m_model.searchLines[ 0 ].bot.x );
            int top = std::min( m_model.searchLines[ 0 ].top.y, m_model.searchLines[ m_model.searchLines.size() - 1 ].top.y );
            int right = std::max( m_model.searchLines[ m_model.searchLines.size() - 1 ].top.x, m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.x );
            int bottom = std::max( m_model.searchLines[ 0 ].bot.y, m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.y );
            rect = Rect( left, top, right - left, bottom - top );
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::GetSearchRegionBoundingRect] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibBowtie::DrawOverlay( const cv::Mat img, cv::Mat &imgOut, const bool drawCalib,
                              const bool drawMoveROIs, const bool drawSearchROI )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( CV_8UC1 == img.type() )
        {
             cvtColor( img, imgOut, COLOR_GRAY2BGR );
        }
        else if ( CV_8UC3 == img.type() )
        {
            imgOut = img.clone();
        }
        else
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::DrawOverlay] Invalid image format for calibration";
            retVal = GC_ERR;
        }

        if ( m_model.pixelPoints.empty() || m_model.worldPoints.empty() || m_model.searchLines.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::DrawOverlay] No valid calibration for drawing";
            retVal = GC_ERR;
        }
        else if ( m_matHomogPixToWorld.empty() || m_matHomogWorldToPix.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::DrawOverlay] System not calibrated";
            retVal = GC_ERR;
        }

        if ( GC_OK == retVal )
        {
            int textOffset = cvRound( static_cast< double >( imgOut.rows ) / 6.6666667 );
            int circleSize =  std::max( 5, cvRound( static_cast< double >( imgOut.rows ) / 120.0 ) );
            int textStroke = std::max( 1, cvRound( static_cast< double >( imgOut.rows ) / 300.0 ) );
            double fontScale = 1.0 + static_cast< double >( imgOut.rows ) / 1200.0;

            if ( drawMoveROIs )
            {
                rectangle( imgOut, m_model.moveSearchRegionLft, Scalar( 0, 0, 255 ), textStroke );
                rectangle( imgOut, m_model.moveSearchRegionRgt, Scalar( 0, 0, 255 ), textStroke );
            }

            if ( drawSearchROI )
            {
                if ( m_model.searchLines.empty() )
                {
                    FILE_LOG( logWARNING ) << "[CalibBowtie::Calibrate] Search lines not calculated properly so they cannot be drawn";
                    retVal = GC_WARN;
                }
                else
                {
                    line( imgOut, m_model.searchLines[ 0 ].top, m_model.searchLines[ 0 ].bot, Scalar( 255, 0, 0 ), textStroke );
                    line( imgOut, m_model.searchLines[ 0 ].top, m_model.searchLines[ m_model.searchLines.size() - 1 ].top, Scalar( 255, 0, 0 ), textStroke );
                    line( imgOut, m_model.searchLines[ m_model.searchLines.size() - 1 ].top, m_model.searchLines[ m_model.searchLines.size() - 1 ].bot, Scalar( 255, 0, 0 ), textStroke );
                    line( imgOut, m_model.searchLines[ 0 ].bot, m_model.searchLines[ m_model.searchLines.size() - 1 ].bot, Scalar( 255, 0, 0 ), textStroke );
                }
            }

            if ( drawCalib )
            {
                Point2d topLft, botRgt;
                retVal = PixelToWorld( m_model.pixelPoints[ 0 ], topLft );
                if ( GC_OK == retVal )
                {
                    retVal = PixelToWorld( m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ], botRgt );
                    if ( GC_OK == retVal )
                    {
                        Point2d pt1, pt2;
                        double minCol = std::min( topLft.x, botRgt.x );
                        double maxCol = std::max( topLft.x, botRgt.x );
                        double minRow = std::min( topLft.y, botRgt.y );
                        double maxRow = std::max( topLft.y, botRgt.y );
                        double rowInc = ( maxRow - minRow ) / static_cast< double >( m_model.gridSize.height + 2 );
                        double colInc = ( maxCol - minCol ) / static_cast< double >( m_model.gridSize.width );
                        minRow -= rowInc;
                        maxRow += rowInc;
                        stringstream buf;

                        bool first;
                        double row, col;
                        int rowInt, colInt;
                        for ( rowInt = 0, row = maxRow; row > minRow; row -= rowInc, ++rowInt )
                        {
                            first = true;
                            for ( colInt = 0, col = minCol; col < maxCol; col += colInc, ++colInt )
                            {
                                retVal = WorldToPixel( Point2d( col, row ), pt1 );
                                if ( GC_OK == retVal )
                                {
                                    retVal = WorldToPixel( Point2d( col + colInc, row ), pt2 );
                                    if ( GC_OK == retVal )
                                    {
                                        line( imgOut, pt1, pt2, Scalar( 0, 255, 255 ), textStroke );
                                        retVal = WorldToPixel( Point2d( col, row - rowInc ), pt2 );
                                        if ( GC_OK == retVal && pt1.y < imgOut.rows )
                                        {
                                            line( imgOut, pt1, pt2, Scalar( 0, 255, 255 ), textStroke );
                                            if ( ( ( rowInt % 2 ) == 1 ) && ( ( colInt % 2 ) == 0 ) )
                                                circle( imgOut, pt1, circleSize, Scalar( 0, 255, 0 ), textStroke );
                                        }
                                    }
                                }
                                if ( first )
                                {
                                    first = false;
                                    buf.str( string() ); buf << boost::format( "%.1f" ) % row;
                                    putText( imgOut, buf.str(), Point( cvRound( pt1.x ) - textOffset, cvRound( pt1.y ) + 5 ),
                                             FONT_HERSHEY_COMPLEX, fontScale * 0.5, Scalar( 0, 255, 255 ), textStroke );
                                }
                            }
                            retVal = WorldToPixel( Point2d( maxCol, row ), pt1 );
                            if ( GC_OK == retVal && pt1.y < imgOut.rows )
                            {
                                retVal = WorldToPixel( Point2d( maxCol, row - rowInc ), pt2 );
                                if ( GC_OK == retVal )
                                {
                                    line( imgOut, pt1, pt2, Scalar( 0, 255, 255 ), textStroke );
                                    if ( ( rowInt % 2 ) == 1 )
                                        circle( imgOut, pt1, circleSize, Scalar( 0, 255, 0 ), textStroke );
                                }
                            }
                        }
                        first = true;
                        for ( double col = minCol; col < maxCol; col += colInc )
                        {
                            retVal = WorldToPixel( Point2d( col, minRow ), pt1 );
                            if ( GC_OK == retVal )
                            {
                                retVal = WorldToPixel( Point2d( col + colInc, minRow ), pt2 );
                                if ( GC_OK == retVal )
                                {
                                    line( imgOut, pt1, pt2, Scalar( 0, 255, 255 ), textStroke );
                                }
                            }
                            if ( first )
                            {
                                first = false;
                                buf.str( string() ); buf << boost::format( "%.1f" ) % minRow;
                                putText( imgOut, buf.str(), Point( cvRound( pt1.x ) - textOffset, cvRound( pt1.y ) + 5 ),
                                         FONT_HERSHEY_COMPLEX, fontScale * 0.5, Scalar( 0, 255, 255 ), textStroke );
                            }
                        }
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::DrawOverlay] " << e.what();
        return GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalibBowtie::PixelToWorld( const Point2d ptPixel, Point2d &ptWorld )
{
    GC_STATUS retVal = GC_OK;

    if ( m_matHomogPixToWorld.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::PixelToWorld] No calibration for pixel to world conversion";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptPixel );
            perspectiveTransform( vecIn, vecOut, m_matHomogPixToWorld );
            ptWorld = vecOut[ 0 ];
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::PixelToWorld] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS CalibBowtie::WorldToPixel( const Point2d ptWorld, Point2d &ptPixel )
{
    GC_STATUS retVal = GC_OK;
    // if ( 9 != m_worldToPixParams.size() )
    if ( m_matHomogWorldToPix.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::WorldToPixel] No calibration for world to pixel conversion";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
#if 1
            vector< Point2d > vecIn, vecOut;
            vecIn.push_back( ptWorld );
            perspectiveTransform( vecIn, vecOut, m_matHomogWorldToPix );
            ptPixel = vecOut[ 0 ];
#else
            double tx = m_worldToPixParams[ 0 ] * ptWorld.x + m_worldToPixParams[ 1 ] * ptWorld.y + m_worldToPixParams[ 2 ];
            double ty = m_worldToPixParams[ 3 ] * ptWorld.x + m_worldToPixParams[ 4 ] * ptWorld.y + m_worldToPixParams[ 5 ];
            double tz = m_worldToPixParams[ 6 ] * ptWorld.x + m_worldToPixParams[ 7 ] * ptWorld.y + m_worldToPixParams[ 8 ];

            ptPixel = Point2d( tx / tz, ty / tz );
#endif

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // https://stackoverflow.com/questions/32366786/opencv-homography-matrix-h-and-inverse-h-to-transform-a-point-is-not-getting-exp
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            //def transformPoints(self, x, y, reverse=False, integer=True):
            //
            //    if reverse == False:
            //        H = self.transform_matrix
            //    else:
            //        val, H = cv2.invert(self.transform_matrix)
            //
            //    # get the elements in the transform matrix
            //    h0 = H[0,0]
            //    h1 = H[0,1]
            //    h2 = H[0,2]
            //    h3 = H[1,0]
            //    h4 = H[1,1]
            //    h5 = H[1,2]
            //    h6 = H[2,0]
            //    h7 = H[2,1]
            //    h8 = H[2,2]
            //
            //    tx = (h0*x + h1*y + h2)
            //    ty = (h3*x + h4*x + h5)
            //    tz = (h6*x + h7*y + h8)
            //
            //    if integer==True:
            //        px = int(tx/tz)
            //        py = int(ty/tz)
            //        Z = int(1/tz)
            //    else:
            //        px = tx/tz
            //        py = ty/tz
            //        Z = 1/tz
            //
            //    return (px, py)
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::WorldToPixel] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
cv::Rect CalibBowtie::MoveSearchROI( const bool isLeft )
{
    return isLeft ? m_model.moveSearchRegionLft :
                    m_model.moveSearchRegionRgt;
}
GC_STATUS CalibBowtie::MoveRefPoint( cv::Point2d &lftRefPt, cv::Point2d &rgtRefPt )
{
    GC_STATUS retVal = GC_OK;
    Point2d pt( numeric_limits< double >::min(), numeric_limits< double >::min() );
    if ( m_model.pixelPoints.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::MoveRefPoint] Cannot retrieve move reference point from an uncalibrated system";
        retVal = GC_ERR;
    }
    else if ( static_cast< size_t >( m_model.gridSize.width * m_model.gridSize.height ) != m_model.pixelPoints.size() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::MoveRefPoint] Cannot retrieve move reference point with invalid calibration";
        retVal = GC_ERR;
    }
    else
    {
        lftRefPt = m_model.pixelPoints[ 0 ];
        rgtRefPt = m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width - 1 ) ];
    }
    return retVal;
}
GC_STATUS CalibBowtie::Load( const string &jsonCalibString )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( jsonCalibString.empty() )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::Load] Bow tie calibration string is empty";
            retVal = GC_ERR;
        }
        else
        {
            stringstream ss;
            ss << jsonCalibString;
            property_tree::ptree ptreeTop;
            property_tree::json_parser::read_json( ss, ptreeTop );

            m_model.imgSize.width = ptreeTop.get< int >( "imageWidth", 0 );
            m_model.imgSize.height = ptreeTop.get< int >( "imageHeight", 0 );
            property_tree::ptree ptreeCalib = ptreeTop.get_child( "PixelToWorld" );

            Point2d ptTemp;
            size_t cols = ptreeCalib.get< size_t >( "columns", 2 );
            size_t rows = ptreeCalib.get< size_t >( "rows", 4 );
            m_model.pixelPoints.clear();
            m_model.worldPoints.clear();

            BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeCalib.get_child( "points" ) )
            {
                ptTemp.x = node.second.get< double >( "pixelX", 0.0 );
                ptTemp.y = node.second.get< double >( "pixelY", 0.0 );
                m_model.pixelPoints.push_back( ptTemp );
                ptTemp.x = node.second.get< double >( "worldX", 0.0 );
                ptTemp.y = node.second.get< double >( "worldY", 0.0 );
                m_model.worldPoints.push_back( ptTemp );
            }

            const property_tree::ptree &ptreeMoveSearch = ptreeTop.get_child( "MoveSearchRegions" );
            property_tree::ptree::const_iterator end = ptreeMoveSearch.end();
            for ( property_tree::ptree::const_iterator iter = ptreeMoveSearch.begin(); iter != end; ++iter )
            {
                if ( iter->first == "Left" )
                {
                    m_model.moveSearchRegionLft.x =      iter->second.get< int >( "x", 0 );
                    m_model.moveSearchRegionLft.y =      iter->second.get< int >( "y", 0 );
                    m_model.moveSearchRegionLft.width =  iter->second.get< int >( "width", 0 );
                    m_model.moveSearchRegionLft.height = iter->second.get< int >( "height", 0 );
                }
                else if ( iter->first == "Right" )
                {
                    m_model.moveSearchRegionRgt.x =      iter->second.get< int >( "x", 0 );
                    m_model.moveSearchRegionRgt.y =      iter->second.get< int >( "y", 0 );
                    m_model.moveSearchRegionRgt.width =  iter->second.get< int >( "width", 0 );
                    m_model.moveSearchRegionRgt.height = iter->second.get< int >( "height", 0 );
                }
                else if ( iter->first == "Target" )
                {
                    m_model.wholeTargetRegion.x =      iter->second.get< int >( "x", 0 );
                    m_model.wholeTargetRegion.y =      iter->second.get< int >( "y", 0 );
                    m_model.wholeTargetRegion.width =  iter->second.get< int >( "width", 0 );
                    m_model.wholeTargetRegion.height = iter->second.get< int >( "height", 0 );
                }
            }

            Point ptTop, ptBot;
            m_model.searchLines.clear();
            BOOST_FOREACH( property_tree::ptree::value_type &node, ptreeTop.get_child( "SearchLines" ) )
            {
                ptTop.x = node.second.get< int >( "topX", std::numeric_limits< int >::min() );
                ptTop.y = node.second.get< int >( "topY", std::numeric_limits< int >::min() );
                ptBot.x = node.second.get< int >( "botX", std::numeric_limits< int >::min() );
                ptBot.y = node.second.get< int >( "botY", std::numeric_limits< int >::min() );
                m_model.searchLines.push_back( LineEnds( ptTop, ptBot ) );
            }

#ifdef LOG_CALIB_VALUES
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Camera calibration association points";
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
            FILE_LOG( logINFO ) << "Columns=" << cols << " Rows=" << rows;
            FILE_LOG( logINFO ) << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~";
#endif
            if ( cols * rows != m_model.pixelPoints.size() )
            {
                FILE_LOG( logERROR ) << "[CalibBowtie::Load] Invalid association point count";
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
                m_model.gridSize = Size( static_cast< int >( cols ), static_cast< int >( rows ) );

                m_model.controlJson = ptreeTop.get< string >( "control_json", "{}" );

                Mat matIn, matOut;
                retVal = Calibrate( m_model.pixelPoints, m_model.worldPoints, m_model.controlJson,
                                    m_model.gridSize, m_model.imgSize );
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
        FILE_LOG( logERROR ) << "[CalibBowtie::Load] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS CalibBowtie::Save( const string jsonCalFilepath )
{
    GC_STATUS retVal = GC_OK;

    if ( m_model.pixelPoints.empty() || m_model.worldPoints.empty() ||
         m_model.pixelPoints.size() != m_model.worldPoints.size() ||
         2 > m_model.gridSize.width || 4 > m_model.gridSize.height || m_model.searchLines.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::Save] Invalid calib grid dimension(s) or empty cal point vector(s)";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            ofstream fileStream( jsonCalFilepath, ios::out );
            if ( fileStream.is_open() )
            {
                fileStream << "{" << endl;
                fileStream << "  \"calibType\":\"BowTie\"" << "," << endl;
                fileStream << "  \"imageWidth\":" << m_model.imgSize.width << "," << endl;
                fileStream << "  \"imageHeight\":" << m_model.imgSize.height << "," << endl;
                fileStream << "  \"PixelToWorld\": " << endl;
                fileStream << "  {" << endl;
                fileStream << "    \"columns\": " << m_model.gridSize.width << "," << endl;
                fileStream << "    \"rows\": " <<    m_model.gridSize.height << "," << endl;
                fileStream << "    \"points\": [" << endl;
                fileStream << fixed << setprecision( 3 );
                for ( size_t i = 0; i < m_model.pixelPoints.size() - 1; ++i )
                {
                    fileStream << "      { \"pixelX\": " << m_model.pixelPoints[ i ].x << ", " << \
                                          "\"pixelY\": " << m_model.pixelPoints[ i ].y << ", " << \
                                          "\"worldX\": " << m_model.worldPoints[ i ].x << ", " << \
                                          "\"worldY\": " << m_model.worldPoints[ i ].y << " }," << endl;
                }
                fileStream << "      { \"pixelX\": " << m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"pixelY\": " << m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].y << ", " << \
                                      "\"worldX\": " << m_model.worldPoints[ m_model.pixelPoints.size() - 1 ].x << ", " << \
                                      "\"worldY\": " << m_model.worldPoints[ m_model.pixelPoints.size() - 1 ].y << " }" << endl;
                fileStream << "    ]" << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"MoveSearchRegions\": " << endl;
                fileStream << "  {" << endl;
                fileStream << fixed << setprecision( 0 );
                fileStream << "    \"Left\":  { " << \
                                    "\"x\": " <<      m_model.moveSearchRegionLft.x << ", " << \
                                    "\"y\": " <<      m_model.moveSearchRegionLft.y << ", " << \
                                    "\"width\": " <<  m_model.moveSearchRegionLft.width << ", " << \
                                    "\"height\": " << m_model.moveSearchRegionLft.height << " }, " << endl;
                fileStream << "    \"Right\": { " << \
                                    "\"x\": " <<      m_model.moveSearchRegionRgt.x << ", " << \
                                    "\"y\": " <<      m_model.moveSearchRegionRgt.y << ", " << \
                                    "\"width\": " <<  m_model.moveSearchRegionRgt.width << ", " << \
                                    "\"height\": " << m_model.moveSearchRegionRgt.height << " }," << endl;
                fileStream << "    \"Target\": { " << \
                                    "\"x\": " <<      m_model.wholeTargetRegion.x << ", " << \
                                    "\"y\": " <<      m_model.wholeTargetRegion.y << ", " << \
                                    "\"width\": " <<  m_model.wholeTargetRegion.width << ", " << \
                                    "\"height\": " << m_model.wholeTargetRegion.height << " }" << endl;
                fileStream << "  }," << endl;
                fileStream << "  \"SearchLines\": [" << endl;
                for ( size_t i = 0; i < m_model.searchLines.size() - 1; ++i )
                {
                    fileStream << "      { \"topX\": " << m_model.searchLines[ i ].top.x << ", " << \
                                          "\"topY\": " << m_model.searchLines[ i ].top.y << ", " << \
                                          "\"botX\": " << m_model.searchLines[ i ].bot.x << ", " << \
                                          "\"botY\": " << m_model.searchLines[ i ].bot.y << " }," << endl;
                }
                fileStream << "      { \"topX\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].top.x << ", " << \
                                      "\"topY\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].top.y << ", " << \
                                      "\"botX\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.x << ", " << \
                                      "\"botY\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.y << " }" << endl;
                fileStream << "  ]," << endl;

                string escaped;
                for ( size_t i = 0; i < m_model.controlJson.size(); ++i )
                {
                    if ( m_model.controlJson[ i ] == '\"' )
                        escaped += "\\\"";
                    else
                        escaped += m_model.controlJson[ i ];
                }

                fileStream << "  \"control_json\": \"" << escaped << "\"" << endl;
                fileStream << "}" << endl;
                fileStream.close();
            }
            else
            {
                FILE_LOG( logERROR ) << "[CalibBowtie::Save] Could not open calibration save file " << jsonCalFilepath;
                retVal = GC_ERR;
            }
        }
        catch( boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::Save] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS CalibBowtie::CalcSearchSwaths()
{
    GC_STATUS retVal = GC_OK;

    if ( m_model.pixelPoints.empty() || m_model.worldPoints.empty() ||
         m_model.pixelPoints.size() != m_model.worldPoints.size() ||
         2 > m_model.gridSize.width || 4 > m_model.gridSize.height )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::CalcSearchSwaths] Invalid calib grid dimension(s) or empty cal point vector(s)";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            int widthTop = cvRound( ( m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width ) - 1 ].x - m_model.pixelPoints[ 0 ].x )  / 3.0 );
            double widthBot = ( m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width * m_model.gridSize.height ) - 1 ].x -
                                m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width * ( m_model.gridSize.height - 1 ) ) ].x ) / 3.0;
            int height = cvRound( ( m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width * ( m_model.gridSize.height - 1 ) ) ].y - m_model.pixelPoints[ 0 ].y ) * 1.25 );
            double topLftX = m_model.pixelPoints[ 0 ].x + static_cast< double >( widthTop );
            double topLftY = m_model.pixelPoints[ 0 ].y - ( static_cast< double >( height ) / 8.0 ) + static_cast< double >( height >> 4 );
            double botLftX = m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width * ( m_model.gridSize.height - 1 ) ) ].x + static_cast< double >( widthBot );
            double botLftY = m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width * ( m_model.gridSize.height - 1 ) ) ].y + static_cast< double >( height ) / 8.0 + static_cast< double >( height >> 4 );
            botLftY = std::min( botLftY, static_cast< double >( m_model.imgSize.height - 1 ) );

            double xInc = 1.0;
            double xIncBot = widthBot / static_cast< double >( widthTop );
            double yInc = ( m_model.pixelPoints[ static_cast< size_t >( m_model.gridSize.width ) - 1 ].y - m_model.pixelPoints[ 0 ].y ) / ( static_cast< double >( height ) * 3.0 );

            m_model.searchLines.clear();
            Point2d ptTop = Point2d( topLftX, topLftY );
            Point2d ptBot = Point2d( botLftX, botLftY );
            for ( int i = 0; i <= widthTop; ++i )
            {
                m_model.searchLines.push_back( LineEnds( Point( cvRound( ptTop.x ), cvRound( ptTop.y ) ),
                                                         Point( cvRound( ptBot.x ), cvRound( ptBot.y ) ) ) );
                ptTop.x += xInc;
                ptTop.y += yInc;
                ptBot.x += xIncBot;
                ptBot.y += yInc;
            }
        }
        catch( boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::CalcSearchSwaths] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
string CalibBowtie::ModelJsonString()
{
    stringstream ss;

    if ( m_model.pixelPoints.empty() || m_model.worldPoints.empty() ||
         m_model.pixelPoints.size() != m_model.worldPoints.size() ||
         2 > m_model.gridSize.width || 4 > m_model.gridSize.height || m_model.searchLines.empty() )
    {
        FILE_LOG( logERROR ) << "[CalibBowtie::SettingsJsonString] Invalid calib grid dimension(s) or empty cal point vector(s)";
    }
    else
    {
        try
        {
            ss << "{" << endl;
            ss << "  \"PixelToWorld\": " << endl;
            ss << "  {" << endl;
            ss << "    \"columns\": " << m_model.gridSize.width << "," << endl;
            ss << "    \"rows\": " <<    m_model.gridSize.height << "," << endl;
            ss << "    \"points\": [" << endl;
            for ( size_t i = 0; i < m_model.pixelPoints.size() - 1; ++i )
            {
                ss << "      { \"pixelX\": " << m_model.pixelPoints[ i ].x << ", " << \
                                      "\"pixelY\": " << m_model.pixelPoints[ i ].y << ", " << \
                                      "\"worldX\": " << m_model.worldPoints[ i ].x << ", " << \
                                      "\"worldY\": " << m_model.worldPoints[ i ].y << " }," << endl;
            }
            ss << "      { \"pixelX\": " << m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].x << ", " << \
                                  "\"pixelY\": " << m_model.pixelPoints[ m_model.pixelPoints.size() - 1 ].y << ", " << \
                                  "\"worldX\": " << m_model.worldPoints[ m_model.pixelPoints.size() - 1 ].x << ", " << \
                                  "\"worldY\": " << m_model.worldPoints[ m_model.pixelPoints.size() - 1 ].y << " }" << endl;
            ss << "    ]" << endl;
            ss << "  }," << endl;
            ss << "  \"MoveSearchRegions\": " << endl;
            ss << "  {" << endl;
            ss << "    \"Left\":  { " << \
                                "\"x\": " <<      m_model.moveSearchRegionLft.x << ", " << \
                                "\"y\": " <<      m_model.moveSearchRegionLft.y << ", " << \
                                "\"width\": " <<  m_model.moveSearchRegionLft.width << ", " << \
                                "\"height\": " << m_model.moveSearchRegionLft.height << " }, " << endl;
            ss << "    \"Right\": { " << \
                                "\"x\": " <<      m_model.moveSearchRegionRgt.x << ", " << \
                                "\"y\": " <<      m_model.moveSearchRegionRgt.y << ", " << \
                                "\"width\": " <<  m_model.moveSearchRegionRgt.width << ", " << \
                                "\"height\": " << m_model.moveSearchRegionRgt.height << " }" << endl;
            ss << "  }," << endl;
            ss << "  \"SearchLines\": [" << endl;
            for ( size_t i = 0; i < m_model.searchLines.size() - 1; ++i )
            {
                ss << "      { \"topX\": " << m_model.searchLines[ i ].top.x << ", " << \
                                      "\"topY\": " << m_model.searchLines[ i ].top.y << ", " << \
                                      "\"botX\": " << m_model.searchLines[ i ].bot.x << ", " << \
                                      "\"botY\": " << m_model.searchLines[ i ].bot.y << " }," << endl;
            }
            ss << "      { \"topX\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].top.x << ", " << \
                                  "\"topY\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].top.y << ", " << \
                                  "\"botX\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.x << ", " << \
                                  "\"botY\": " << m_model.searchLines[ m_model.searchLines.size() - 1 ].bot.y << " }" << endl;
            ss << "  ]" << endl;
            ss << "}" << endl;
        }
        catch( boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalibBowtie::SettingsJsonString] " << diagnostic_information( e );
        }
    }

    return ss.str();
}

} // namespace gc
