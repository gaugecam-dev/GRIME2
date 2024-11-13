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
#include "octagonsearch.h"
#include <random>
#include <iostream>
#include <algorithm>
#include <chrono>
#include "opencv2/imgcodecs.hpp"
#include "bresenham.h"

#ifdef DEBUG_STOPSIGN_TEMPL
#undef DEBUG_STOPSIGN_TEMPL
#include <boost/filesystem.hpp>
using namespace boost;
namespace fs = filesystem;
#ifdef _WIN32
static const char DEBUG_FOLDER[] = "c:/gaugecam/";
#else
static const char DEBUG_FOLDER[] = "/var/tmp/gaugecam/";
#endif
#endif

using namespace cv;
using namespace std;

namespace gc
{

OctagonSearch::OctagonSearch()
{
#ifdef DEBUG_STOPSIGN_TEMPL
    if ( !fs::exists( DEBUG_FOLDER ) )
    {
        bool ret = fs::create_directories( DEBUG_FOLDER );
        if ( !ret )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::StopsignSearch] Could not create debug result folder " << DEBUG_FOLDER;
        }
    }
#endif
}
GC_STATUS OctagonSearch::FindScale( const cv::Mat &img, std::vector< cv::Point2d > &pts, const double scale, const bool do_coarse_prefind )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        pts.clear();
        Mat imgScaled;
        resize( img, imgScaled, Size(), scale, scale, INTER_CUBIC );
        retVal = Find( imgScaled, pts, do_coarse_prefind );
        if ( GC_OK == retVal )
        {
            for ( size_t i = 0; i < pts.size(); ++i )
            {
                pts[ i ] *= ( 1.0 / scale );
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::FindScale] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
//GC_STATUS StopsignSearch::AdjustSearchImage( const Mat &img, Mat &adjusted )
//{
//    GC_STATUS retVal = GC_OK;
//    try
//    {
//        if ( img.empty() )
//        {
//            FILE_LOG( logERROR ) << "[StopsignSearch::AdjustRawImage] Empty raw image";
//            retVal = GC_ERR;
//        }
//        else
//        {
//#ifdef DEBUG_STOPSIGN_TEMPL
//            imwrite( "/var/tmp/gaugecam/raw_image.png", img );
//#endif
//            Mat mask;
//            double thr = threshold( img, mask, 1, 255, THRESH_OTSU );
//            threshold( img, mask, std::max( 0.0, thr * 1.1 ), 255, THRESH_BINARY );
//            Mat kern = getStructuringElement( MORPH_ELLIPSE, Size( 5, 5 ) );
//            erode( mask, mask, kern );
//#ifdef DEBUG_STOPSIGN_TEMPL
//            imwrite( "/var/tmp/gaugecam/raw_threshed.png", mask );
//#endif
//            int idx = -1;
//            double area, area_max =  -999999999;
//            vector< vector< Point > > contours;
//            findContours( mask, contours, RETR_LIST, CHAIN_APPROX_SIMPLE );
//            for ( size_t i = 0; i < contours.size(); ++i )
//            {
//                area = contourArea( contours[ i ] );
//                if ( area_max < area )
//                {
//                    area_max = area;
//                    idx = static_cast< int >( i );
//                }
//            }
//            if ( 0 > idx )
//            {
//                mask.setTo( 255 );
//            }
//            else
//            {
//                Mat scratch;
//                img.copyTo( scratch );
//                mask.setTo( 255 );
//                drawContours( mask, vector< vector< Point > >( 1, contours[ idx ] ), -1, Scalar( 0 ), FILLED );
//                adjusted = mask | img;
//            }
//#ifdef DEBUG_STOPSIGN_TEMPL
//            imwrite( "/var/tmp/gaugecam/raw_mask.png", mask );
//            imwrite( "/var/tmp/gaugecam/raw_adjusted.png", adjusted );
//#endif
//        }
//    }
//    catch( const cv::Exception &e )
//    {
//        FILE_LOG( logERROR ) << "[AdjustRawImage::FindScale] " << e.what();
//        retVal = GC_EXCEPT;
//    }
//    return retVal;
//}
GC_STATUS OctagonSearch::CoarseOctoMask( const Mat &img, Mat &mask )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::AdjustRawImage] Empty raw image";
            retVal = GC_ERR;
        }
        else
        {
            Mat response, response_8u;
            int maxRadIdx = -1;
            double maxMaxVal = -9999999;

            Point maxPt;
            double maxVal;
            Point2d maxMaxPt;

            mask = Mat::ones( img.size(), CV_8UC1 ) * 255;
            for ( size_t i = 0; i < octoTemplates.templates.size(); ++i )
            {
                response = Mat();
#ifdef DEBUG_STOPSIGN_TEMPL
                imwrite("/var/tmp/gaugecam/response_templ.png", octoTemplates.templates[ i ].templ );
                imwrite("/var/tmp/gaugecam/response_mask.png", octoTemplates.templates[ i ].mask );
#endif
                matchTemplate( img, octoTemplates.templates[ i ].templ, response, TM_CCORR_NORMED, octoTemplates.templates[ i ].mask );
#ifdef DEBUG_STOPSIGN_TEMPL
                normalize( response, response_8u, 0, 1, NORM_MINMAX );
                response_8u.convertTo( response_8u, CV_8UC1, 255 );
                imwrite("/var/tmp/gaugecam/response.png", response_8u);
#endif

                minMaxLoc( response, nullptr, &maxVal, nullptr, &maxPt );
                maxVal *= octoTemplates.templates[ i ].mask_pix_count;
                if ( maxVal > maxMaxVal )
                {
                    maxRadIdx = static_cast< int >( i );
                    maxMaxVal = maxVal;
                    maxMaxPt = Point2d( maxPt ) + octoTemplates.templates[ i ].offset;
                }
            }
            if ( -1 != maxRadIdx )
            {
                mask = Mat::zeros( img.size(), CV_8UC1 );
                circle( mask, maxMaxPt, octoTemplates.templates[ maxRadIdx ].radius, Scalar(255), octoTemplates.templates[ maxRadIdx ].thickness );
#ifdef DEBUG_STOPSIGN_TEMPL
                Mat color;
                cvtColor( img, color, COLOR_GRAY2BGR );
                line( color, Point( cvRound( maxMaxPt.x - 10 ), cvRound( maxMaxPt.y ) ),
                      Point( cvRound( maxMaxPt.x + 10 ), cvRound( maxMaxPt.y ) ), Scalar( 0, 255, 255 ), 1 );
                line( color, Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y - 10 ) ),
                      Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y + 10 ) ), Scalar( 0, 255, 255 ), 1 );
                circle( color, maxMaxPt, octoTemplates.templates[ maxRadIdx ].radius - ( octoTemplates.templates[ maxRadIdx ].thickness >> 1 ), Scalar(0,0,255), 3);
                circle( color, maxMaxPt, octoTemplates.templates[ maxRadIdx ].radius + ( octoTemplates.templates[ maxRadIdx ].thickness >> 1 ), Scalar(0,0,255), 3);
                imwrite( "/var/tmp/gaugecam/coarseFind1.png", color );
#endif
            }
#ifdef DEBUG_STOPSIGN_TEMPL
            Mat color;
            cvtColor( img, color, COLOR_GRAY2BGR );
            line( color, Point( cvRound( maxMaxPt.x - 10 ), cvRound( maxMaxPt.y ) ),
                  Point( cvRound( maxMaxPt.x + 10 ), cvRound( maxMaxPt.y ) ), Scalar( 0, 255, 255 ), 1 );
            line( color, Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y - 10 ) ),
                  Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y + 10 ) ), Scalar( 0, 255, 255 ), 1 );
            imwrite( "/var/tmp/gaugecam/coarseFind_mask.png", mask );
            imwrite( "/var/tmp/gaugecam/coarseFind_img.png", img );
            imwrite( "/var/tmp/gaugecam/coarseFind.png", color );
#endif
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[AdjustRawImage::FindScale] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::AdjustResponseSpace( Mat &response, const size_t j )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( response.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::AdjustResponseSpace] Empty response space image";
            retVal = GC_ERR;
        }
        else
        {
#ifdef DEBUG_STOPSIGN_TEMPL
            Mat scratch;
            normalize( response, scratch, 0, 1, NORM_MINMAX );
            scratch.convertTo( scratch, CV_8UC1, 255 );
            imwrite("/var/tmp/gaugecam/response.png", scratch);
#endif
            Mat mask = Mat::zeros( response.size(), CV_8UC1 );
            vector< Point > contour;
            switch( j )
            {
                case 0:
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    break;
                case 1:
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    break;
                case 2:
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    break;
                case 3:
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    break;
                case 4:
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    break;
                case 5:
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    break;
                case 6:
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    break;
                case 7:
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    break;
                default:
                    contour.push_back( Point( 0, 0 ) );
                    contour.push_back( Point( 0, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, mask.rows - 1 ) );
                    contour.push_back( Point( mask.cols - 1, 0 ) );
                    break;
            }
            drawContours( mask, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), FILLED );
            drawContours( mask, vector< vector< Point > >( 1, contour ), -1, Scalar( 0 ), 7 );
            response.setTo( 0, mask == 255 );
#ifdef DEBUG_STOPSIGN_TEMPL
            normalize( response, scratch, 0, 1, NORM_MINMAX );
            scratch.convertTo( scratch, CV_8UC1, 255 );
            imwrite("/var/tmp/gaugecam/response_adj.png", scratch);
            double maxVal;
            Point maxPt;
            minMaxLoc( response, nullptr, &maxVal, nullptr, &maxPt );
            Mat color;
            cvtColor( scratch, color, COLOR_GRAY2BGR );
            line( color, Point( cvRound( maxPt.x - 10 ), cvRound( maxPt.y ) ),
                  Point( cvRound( maxPt.x + 10 ), cvRound( maxPt.y ) ), Scalar( 0, 0, 255 ), 1 );
            line( color, Point( cvRound( maxPt.x ), cvRound( maxPt.y - 10 ) ),
                  Point( cvRound( maxPt.x ), cvRound( maxPt.y + 10 ) ), Scalar( 0, 0, 255 ), 1 );
            imwrite("/var/tmp/gaugecam/response_max.png", color);
#endif
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::AdjustResponseSpace] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::Find( const cv::Mat &img, std::vector< cv::Point2d > &pts, const bool do_coarse_prefind )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::Find] Empty input image";
            retVal = GC_ERR;
        }
        else
        {
            if ( templates.empty() )
            {
                retVal = Init( GC_OCTAGON_TEMPLATE_DIM, 5 );
            }
            if ( templates.empty() || GC_OK != retVal )
            {
                FILE_LOG( logERROR ) << "[StopsignSearch::FindMoveTargets] Cannot find move stop sign in an uninitialized object";
                retVal = GC_ERR;
            }
            else
            {
                Mat matIn = img;
                if ( img.type() == CV_8UC3 )
                    cvtColor( img, matIn, COLOR_BGR2GRAY );

                Mat  mask = Mat::ones( img.size(), CV_8UC1 ) * 255;

                int radBeg = static_cast< int >( round( std::min( img.cols, img.rows ) * 0.2 ) );
                int radEnd = static_cast< int >( round( std::min( img.cols, img.rows ) * 0.45 ) );
                int radInc = static_cast< int >( round( ( radEnd - radBeg ) / 20.0 ) );

                if ( do_coarse_prefind )
                {
                    if ( octoTemplates.templates.empty() )
                    {
                        retVal = CreateOctoTemplates( radBeg, radEnd, radInc, 50, octoTemplates.templates );
                        if ( GC_OK == retVal )
                        {
                            retVal = CoarseOctoMask( matIn, mask );
                        }
#ifdef DEBUG_STOPSIGN_TEMPL
                        imwrite( "/var/tmp/gaugecam/response_mask.png", mask );
#endif
                    }
                }

#ifdef DEBUG_STOPSIGN_TEMPL
                Mat color;
                if ( img.type() == CV_8UC1 )
                    cvtColor( matIn, color, COLOR_GRAY2BGR );
                else
                    img.copyTo( color );
                imwrite("/var/tmp/gaugecam/img_in_find.png", matIn);
#endif

                pts.clear();
                Mat response;
                for ( size_t j = 0; j < templates.size(); ++j )
                {
                    Point2d maxMaxPt;
                    double maxMaxVal = -9999999;
                    for ( size_t i = 0; i < templates[ j ].ptTemplates.size(); ++i )
                    {
                        matchTemplate( matIn, templates[ j ].ptTemplates[ i ].templ, response, TM_CCORR_NORMED, templates[ j ].ptTemplates[ i ].mask );
#ifdef DEBUG_STOPSIGN_TEMPL
                        imwrite("/var/tmp/gaugecam/response_001.png", response);
#endif
                        int l = ( mask.cols - response.cols ) >> 1;
                        int r = ( mask.rows - response.rows ) >> 1;
                        response.setTo( 0.0, mask( Rect( l, r, response.cols, response.rows ) ) == 0 );
#ifdef DEBUG_STOPSIGN_TEMPL
                        imwrite("/var/tmp/gaugecam/response_mask.tiff", response);
#endif
                        retVal = AdjustResponseSpace( response, j );


                        double maxVal;
                        Point maxPt;
                        minMaxLoc( response, nullptr, &maxVal, nullptr, &maxPt );
                        if ( maxVal > maxMaxVal )
                        {
                            maxMaxVal = maxVal;
                            maxMaxPt = Point2d( maxPt ) + templates[ j ].ptTemplates[ i ].offset;
                        }
#ifdef DEBUG_STOPSIGN_TEMPL
                        normalize( response, response, 0, 1, NORM_MINMAX );
                        response.convertTo( response, CV_8UC1, 255 );
                        imwrite("/var/tmp/gaugecam/response.png", response);
#endif
                    }
                    if ( 0.0 < maxMaxVal )
                    {
                        pts.push_back( maxMaxPt );
#ifdef DEBUG_STOPSIGN_TEMPL
                        line( color, Point( cvRound( maxMaxPt.x - 10 ), cvRound( maxMaxPt.y ) ),
                              Point( cvRound( maxMaxPt.x + 10 ), cvRound( maxMaxPt.y ) ), Scalar( 0, 255, 255 ), 1 );
                        line( color, Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y - 10 ) ),
                              Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y + 10 ) ), Scalar( 0, 255, 255 ), 1 );
#endif
                    }
                }

                std::vector< cv::Point2d > pts_temp = pts;
                retVal = RefineFind( img, pts );
                if ( GC_OK != retVal )
                {
                    pts = pts_temp;
                    retVal = GC_OK;
                }

#ifdef DEBUG_STOPSIGN_TEMPL
                imwrite( string(DEBUG_FOLDER) + "stopsign_find_template.png", color );
                for ( size_t i = 0; i < pts.size(); ++i )
                {
                    line( color, Point( cvRound( pts[ i ].x - 10 ), cvRound( pts[ i ].y ) ),
                          Point( cvRound( pts[ i ].x + 10 ), cvRound( pts[ i ].y ) ), Scalar( 0, 255, 0 ), 1 );
                    line( color, Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y - 10 ) ),
                          Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y + 10 ) ), Scalar( 0, 255, 0 ), 1 );
                }
                imwrite( string(DEBUG_FOLDER) + "stopsign_find_line.png", color );
#endif
            }
            if ( 8 != pts.size() )
            {
                FILE_LOG( logERROR ) << "[StopsignSearch::Find] Found only " << pts.size() << " points";
                retVal = GC_ERR;
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::Find] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::FindMoveTargets( const Mat &img, const Rect targetRoi, Point2d &ptLeft, Point2d &ptRight )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::FindMoveTargets] Cannot find move targets in an empty image";
            retVal = GC_ERR;
        }
        else
        {
            if ( templates.empty() )
            {
                retVal = Init( GC_OCTAGON_TEMPLATE_DIM, 7 );
            }
            if ( GC_OK != retVal || templates.empty() )
            {
                FILE_LOG( logERROR ) << "[StopsignSearch::FindMoveTargets] Cannot find move targets in an uninitialized object";
                retVal = GC_ERR;
            }
            else
            {
                Mat matIn = img( targetRoi );
                if ( img.type() == CV_8UC3 )
                    cvtColor( matIn, matIn, COLOR_BGR2GRAY );

                cv::Ptr< CLAHE > clahe = createCLAHE( 1.0 );
                clahe->apply( matIn, matIn );
#ifdef DEBUG_STOPSIGN_TEMPL
                imwrite( string(DEBUG_FOLDER) + "template_stopsign_clahe.png", matIn );
#endif

                // GaussianBlur( matIn, matIn, Size( 5, 5 ), 1.0 );
                medianBlur( matIn, matIn, 7 );
#ifdef DEBUG_STOPSIGN_TEMPL
                imwrite( string(DEBUG_FOLDER) + "template_stopsign_median.png", matIn );
#endif

#ifdef DEBUG_STOPSIGN_TEMPL
                Mat color;
                if ( img.type() == CV_8UC1 )
                    cvtColor( matIn, color, COLOR_BGR2GRAY );
                else
                    img.copyTo( color );
#endif

                ptLeft = Point2d( -1.0, -1.0 );
                ptRight = Point2d( -1.0, -1.0 );

                Point maxPt;
                Mat response;
                Point2d maxMaxPt;
                double maxVal, maxMaxVal;
                vector< int > templIdx = { 0, 7 }; // 0 = top left corner, 7 = top right corner

                for ( size_t j = 0; j < templIdx.size(); ++j )
                {
                    maxMaxVal = -9999999;
                    // imwrite( string(DEBUG_FOLDER) + "stopsign_move_target" + to_string(j) + ".png", templates[ j ].ptTemplates[ 5 ].templ );
                    for ( size_t i = 0; i < templates[ j ].ptTemplates.size(); ++i )
                    {
                        matchTemplate( matIn, templates[ j ].ptTemplates[ i ].templ, response, TM_CCORR_NORMED, templates[ j ].ptTemplates[ i ].mask );

                        minMaxLoc( response, nullptr, &maxVal, nullptr, &maxPt );
                        if ( maxVal > maxMaxVal )
                        {
                            maxMaxVal = maxVal;
                            maxMaxPt = Point2d( maxPt ) + templates[ j ].ptTemplates[ i ].offset;
                            if ( 0 == j )
                            {
                                ptLeft = maxMaxPt;
                            }
                            else
                            {
                                ptRight = maxMaxPt;
                            }
                        }
                    }
                    if ( 0.0 < maxMaxVal )
                    {
#ifdef DEBUG_STOPSIGN_TEMPL
                        line( color( targetRoi ), Point( cvRound( maxMaxPt.x - 10 ), cvRound( maxMaxPt.y ) ),
                              Point( cvRound( maxMaxPt.x + 10 ), cvRound( maxMaxPt.y ) ), Scalar( 0, 255, 0 ), 1 );
                        line( color( targetRoi ), Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y - 10 ) ),
                              Point( cvRound( maxMaxPt.x ), cvRound( maxMaxPt.y + 10 ) ), Scalar( 0, 255, 0 ), 1 );
                        imwrite( string(DEBUG_FOLDER) + "stopsign_move_target_found.png", color );
#endif
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[StopsignSearch::FindMoveTargets] Could not find move target";
                        retVal = GC_ERR;
                        break;
                    }
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::FindMoveTargets] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS OctagonSearch::ShortenLine( const LineEnds &a, const double newLengthPercent, LineEnds &newLine )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double lineLength = sqrt( ( a.top.x - a.bot.x ) * ( a.top.x - a.bot.x ) +
                                  ( a.top.y - a.bot.y ) * ( a.top.y - a.bot.y ) );
        double newLength = lineLength * ( newLengthPercent + ( 1.0 - newLengthPercent ) / 2 );

        retVal = AdjustLineLength( a, newLength, newLine );
        if ( GC_OK == retVal )
        {
            newLength = lineLength * newLengthPercent;
            retVal = AdjustLineLength( LineEnds( newLine.bot, newLine.top ), newLength, newLine );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::ShortenLine] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::AdjustLineLength( const LineEnds &a, const double newLength, LineEnds &newLine )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        newLine.top = a.top;
        double lenAB = sqrt( pow( a.top.x - a.bot.x, 2.0 ) + pow( a.top.y - a.bot.y, 2.0 ) );
        newLine.bot.x = cvRound( a.top.x + ( ( a.top.x - a.bot.x ) / lenAB ) * newLength );
        newLine.bot.y = cvRound( a.top.y + ( ( a.top.y - a.bot.y ) / lenAB ) * newLength );
        newLine.bot.x = cvRound( a.top.x + ( ( a.bot.x - a.top.x ) / lenAB ) * newLength );
        newLine.bot.y = cvRound( a.top.y + ( ( a.bot.y - a.top.y ) / lenAB ) * newLength );

        // lenAB = sqrt(pow(A.x - B.x, 2.0) + pow(A.y - B.y, 2.0));
        // C.x = B.x + (B.x - A.x) / lenAB * length;
        // C.y = B.y + (B.y - A.y) / lenAB * length;
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::AdjustLineLength] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::RefineFind( const Mat &img, vector< Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] Reference image empty";
            retVal = GC_ERR;
        }
        else if ( 8 != pts.size() && 2 != pts.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] Need 8 points, but got only " << pts.size();
            retVal = GC_ERR;
        }
        else
        {
            Mat img8u;
            if ( img.type() == CV_8UC1 )
            {
                img.copyTo( img8u );
            }
            else
            {
                cvtColor( img, img8u, COLOR_BGR2GRAY );
            }

            Mat edges;
            medianBlur( img8u, edges, 7 );
            Canny( edges, edges, 35, 70 );

            LineEnds lineEnds;
            vector< LineEnds > lineEndSet;
            vector< Point > lineEdges;
            Mat mask = Mat::zeros( img.size(), CV_8UC1 );

#ifdef DEBUG_STOPSIGN_TEMPL
            Mat color;
            if ( img.type() == CV_8UC3 )
            {
                img.copyTo( color );
            }
            else
            {
                cvtColor( img, color, COLOR_GRAY2BGR );
            }
#endif

            LineEnds lineA;
            vector< LineEnds > lineSet;
            retVal = ShortenLine( LineEnds( pts[ 0 ], pts[ pts.size() - 1 ] ), 0.9, lineA );
            if ( GC_OK == retVal )
            {
                lineSet.push_back( lineA );
                for ( size_t i = 1; i < pts.size(); ++i )
                {
                    retVal = ShortenLine( LineEnds( pts[ i ], pts[ i - 1 ] ), 0.9, lineA );
                    if ( GC_OK == retVal )
                    {
                        lineSet.push_back( lineA );
                    }
                    else
                    {
                        break;
                    }
                }
                if ( GC_OK == retVal )
                {
                    for ( size_t i = 0; i < lineSet.size(); ++i )
                    {
#ifdef DEBUG_STOPSIGN_TEMPL
                        line( color, lineSet[ i ].bot, lineSet[ i ].top, Scalar( 0, 0, 255 ), 1 );
#endif
                        mask = 0;
                        line( mask, lineSet[ i ].top, lineSet[ i ].bot, Scalar( 255 ), 15 );
                        mask &= edges;
                        // imwrite( "/var/tmp/gaugecam/line_edges.png", mask );

                        findNonZero( mask, lineEdges );

#ifdef DEBUG_STOPSIGN_TEMPL
                        for ( size_t j = 0; j < lineEdges.size(); ++j )
                        {
                            color.at<Vec3b>( lineEdges[ j ] )[ 0 ] = 0;
                            color.at<Vec3b>( lineEdges[ j ] )[ 1 ] = 255;
                            color.at<Vec3b>( lineEdges[ j ] )[ 2 ] = 255;
                        }
#endif

                        retVal = FitLine( lineEdges, lineEnds, img );
                        if ( GC_OK == retVal )
                        {
                            lineEndSet.push_back( lineEnds );
                        }
                        else
                        {
                            break;
                        }
                    }

                    if ( GC_OK == retVal )
                    {
                        retVal = CalcPointsFromLines( lineEndSet, pts );

#ifdef DEBUG_STOPSIGN_TEMPL
                        if ( GC_OK == retVal )
                        {
                            for ( size_t i = 0; i < lineEndSet.size(); ++i )
                            {
                                line( color, Point( cvRound( pts[ i ].x - 10 ), cvRound( pts[ i ].y ) ),
                                      Point( cvRound( pts[ i ].x + 10 ), cvRound( pts[ i ].y ) ), Scalar( 255, 0, 0 ), 1 );
                                line( color, Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y - 10 ) ),
                                      Point( cvRound( pts[ i ].x ), cvRound( pts[ i ].y + 10 ) ), Scalar( 255, 0, 0 ), 1 );
                                line( color, lineEndSet[ i ].bot, lineEndSet[ i ].top, Scalar( 0, 255, 0 ), 1 );
                            }
                            imwrite( "/var/tmp/gaugecam/nighttime.png", color );
                        }
#endif
                    }
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RefineFind] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::Init( const int templateDim, const int rotateCnt )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        templates.clear();
        templates.push_back( OctagonTemplateSet( 0 ) );
        for ( size_t i = 1; i < 8; ++i )
        {
            templates.push_back( OctagonTemplateSet( 360 - static_cast< int >( i ) * 45 ) );
        }

        retVal = CreatePointTemplates( templateDim, rotateCnt, templates[ 0 ].ptTemplates );
        if ( GC_OK == retVal )
        {
            for ( int i = 1; i < 8; ++i )
            {
                retVal = RotatePointTemplates( i, static_cast< double >( 360 - i * 45 ) );
                if ( GC_OK != retVal )
                {
                    break;
                }
            }
        }
//#ifdef DEBUG_STOPSIGN_TEMPL
//        if ( GC_OK == retVal )
//        {
//            retVal = CreateTemplateOverlay( DEBUG_FOLDER );
//        }
//#endif
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::Init] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::RotatePointTemplates( const size_t idx, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( templates.size() <= idx )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] Target template does not exist";
            retVal = GC_ERR;
        }
        else if ( templates[ 0 ].ptTemplates.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] Reference template zero not initialized";
            retVal = GC_ERR;
        }
        else
        {
            Mat rotMask, rotTempl;
            for ( size_t i = 0; i < templates[ 0 ].ptTemplates.size(); ++i )
            {
                templates[ idx ].pointAngle = cvRound( angle );
                templates[ idx ].ptTemplates.push_back( OctagonTemplate() );
                templates[ idx ].ptTemplates[ i ].angle = templates[ 0 ].ptTemplates[ i ].angle;
                templates[ idx ].ptTemplates[ i ].offset = templates[ 0 ].ptTemplates[ i ].offset;
                retVal = RotateImage( templates[ 0 ].ptTemplates[ i ].mask, rotMask, angle );
                if ( GC_OK == retVal )
                {
                    templates[ idx ].ptTemplates[ i ].mask = rotMask.clone();
                    retVal = RotateImage( templates[ 0 ].ptTemplates[ i ].templ, rotTempl, angle );
                    if ( GC_OK == retVal )
                    {
                        templates[ idx ].ptTemplates[ i ].templ = rotTempl.clone();
                    }
                }
                if ( GC_OK != retVal )
                {
                    break;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RotatePointTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::CreateOctoTemplates( const int radBeg, const int radEnd, const int radInc,
                                               const int beg_thickness, std::vector< OctoTemplate > &ptTemplates )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ptTemplates.clear();

        if ( 40 > radBeg || radEnd < radBeg )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreateOctoTemplates] Invalid radius range: start=" << radBeg << " end=" << radEnd << " -- Must be > 50";
            retVal = GC_ERR;
        }
        else
        {
            int templDim = radEnd * 2 + 2;
            templDim = 0 == templDim % 2 ? templDim + 1 : templDim;
            Point center( templDim >> 1, templDim >> 1 );

            int thickness_adj;
            Point2d offset;

            for ( int radius = radBeg; radius <= radEnd; radius += radInc )
            {
                OctoTemplate ssTempScratch;
                thickness_adj = static_cast< int >( std::round( ( beg_thickness * radius ) / static_cast< double >( radBeg ) ) );
                retVal = DrawOctagon( templDim, radius, thickness_adj, ssTempScratch.templ, ssTempScratch.mask, offset );
                if ( GC_OK == retVal )
                {
                    ssTempScratch.radius = radius;
                    ssTempScratch.thickness = thickness_adj;
                    ssTempScratch.offset = center;
                    ssTempScratch.mask_pix_count = countNonZero( ssTempScratch.mask );
                    ptTemplates.push_back( ssTempScratch );
                }
                else
                {
                    break;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CreateOctoTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::CreatePointTemplates( const int templateDim, const int rotateCnt, std::vector< OctagonTemplate > &ptTemplates )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 1 > rotateCnt )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Must have more than one rotation template each direction";
            retVal = GC_ERR;
        }
        else if ( 15 > templateDim )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Template dimension must be at least 15";
            retVal = GC_ERR;
        }
        else
        {
            ptTemplates.clear();
            size_t templCnt = rotateCnt * 2 + 1;
            for ( size_t i = 0; i < templCnt; ++i )
            {
                ptTemplates.push_back( OctagonTemplate() );
            }
            int templDim = templateDim + ( 0 == templateDim % 2 ? 1 : 0 );

            double angle;
            Point2d offset;
            Mat mask, templ, templZero, maskZero;
            retVal = DrawCorner( templDim, templZero, maskZero, offset );
            if ( GC_OK == retVal )
            {
                for ( int i = 0; i < rotateCnt; ++i )
                {
                    retVal = RotateImage( maskZero, mask, static_cast< double >( rotateCnt - i ) );
                    if ( GC_OK == retVal )
                    {
                        threshold( mask, mask, 1, 255, THRESH_BINARY );
                        // imwrite( "/var/tmp/gaugecam/mask_" + to_string( i ) + ".png", mask );
                        retVal = RotateImage( templZero, templ, static_cast< double >( rotateCnt - i ) );
                        if ( GC_OK == retVal )
                        {
                            // imwrite( "/var/tmp/gaugecam/templ_" + to_string( i ) + ".png", templ );
                            // imwrite( "/var/tmp/gaugecam/mask_" + to_string( i ) + ".png", mask );
                            ptTemplates[ i ].mask = mask.clone();
                            ptTemplates[ i ].templ = templ.clone();
                            ptTemplates[ i ].angle = static_cast< double >( rotateCnt - i );
                            ptTemplates[ i ].offset = offset;
                            angle = static_cast< double >( ( rotateCnt - 1 ) - ( i + rotateCnt ) );
                            retVal = RotateImage( maskZero, mask, angle );
                            if ( GC_OK == retVal )
                            {
                                threshold( mask, mask, 1, 255, THRESH_BINARY );
                                retVal = RotateImage( templZero, templ, angle );
                                if ( GC_OK == retVal )
                                {
                                    // imwrite( "/var/tmp/gaugecam/templ_" + to_string( i ) + ".png", templ );
                                    // imwrite( "/var/tmp/gaugecam/mask_" + to_string( i ) + ".png", mask );
                                    ptTemplates[ rotateCnt + i + 1 ].mask = mask.clone();
                                    ptTemplates[ rotateCnt + i + 1 ].templ = templ.clone();
                                    ptTemplates[ rotateCnt + i + 1 ].angle = angle;
                                    ptTemplates[ rotateCnt + i + 1 ].offset = offset;
                                }
                            }
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        break;
                    }
                    // cout << "idx=" << ( i ) << " angle=" << ( 5 - i ) << " idx=" << ( rotateCnt + i + 1 ) << " angle=" << angle << endl;
                }
                if ( GC_OK == retVal )
                {
                    // imwrite( "/var/tmp/gaugecam/mask_zero.png", maskZero );
                    // imwrite( "/var/tmp/gaugecam/templ_zero.png", templZero );
                    ptTemplates[ rotateCnt ].mask = maskZero.clone();
                    ptTemplates[ rotateCnt ].templ = templZero.clone();
                    ptTemplates[ rotateCnt ].angle = 0.0;
                    ptTemplates[ rotateCnt ].offset = offset;
                }
                else
                {
                    FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] Could not rotate templates";
                    retVal = GC_ERR;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CreatePointTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::DrawOctagon( const int templateDim, const int radius, const int thickness, cv::Mat &templ, cv::Mat &mask, Point2d &center )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 30 > templateDim )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::DrawOctagon] Octo template dimension too small=" << templateDim;
            retVal = GC_ERR;
        }
        else
        {
            templ = Mat::ones( Size( templateDim, templateDim ), CV_8UC1 ) * 255;

            center = cv::Point2d( templ.cols >> 1, templ.rows >> 1 );
            circle( templ, center, radius, Scalar( 0 ), thickness );
            Mat kern = getStructuringElement( MORPH_ELLIPSE, Size( 11, 11 ) );
            dilate( ~templ, mask, kern, Point( -1, -1 ), 7 );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::DrawOctagon] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::DrawCorner( const int templateDim, cv::Mat &templ, cv::Mat &mask, Point2d &center )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 30 > templateDim || 0 == templateDim % 2 )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::DrawCorner] Template dimension too small or not odd: dim=" << templateDim;
            retVal = GC_ERR;
        }
        else
        {
            int blackLineWidth = 7;
            int tempRotDim = cvRound( static_cast< double >( templateDim ) * 1.415 );
            tempRotDim += ( 0 == tempRotDim % 2 ? 1 : 0 );
            int rectTopAndLeft = ( tempRotDim - templateDim ) >> 1;
            Rect rect( rectTopAndLeft, rectTopAndLeft, templateDim, templateDim );
            center = Point2d( static_cast< double >( tempRotDim ) / 2.0,
                              static_cast< double >( tempRotDim ) / 2.0 );

            mask = Mat::zeros( Size( tempRotDim, tempRotDim ), CV_8UC1 );

            int ortho_dist = cvRound( blackLineWidth * sin( CV_PI * ( 135.0 / 180.0 ) ) / 2.0 );
            int opposite = cvRound( sqrt( 2.0 * blackLineWidth * blackLineWidth ) );

            vector< Point > contour;
            contour.push_back( Point( ( templateDim >> 1 ) - ortho_dist, ( templateDim >> 1 ) - blackLineWidth ) );
            contour.push_back( Point( 0, templateDim - opposite ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, ( ( templateDim >> 1 ) - blackLineWidth ) ) );
            contour.push_back( Point( ( templateDim >> 1 ) - ortho_dist, ( templateDim >> 1 ) - blackLineWidth ) );
            drawContours( mask( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), FILLED );
            // imwrite( DEBUG_FOLDER + "mask_000.png", mask );

            contour.clear();
            contour.push_back( Point( ( templateDim >> 1 ) + ortho_dist, ( templateDim >> 1 ) + blackLineWidth ) );
            contour.push_back( Point( 0, templateDim + opposite ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, ( ( templateDim >> 1 ) + blackLineWidth ) ) );
            contour.push_back( Point( ( templateDim >> 1 ) + ortho_dist, ( templateDim >> 1 ) + blackLineWidth ) );
            drawContours( mask( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 0 ), FILLED );
            // imwrite( "/var/tmp/gaugecam/mask_001.png", mask );

            contour.clear();
            contour.push_back( Point( ( templateDim >> 1 ), ( templateDim >> 1 ) ) );
            contour.push_back( Point( 0, templateDim ) );
            contour.push_back( Point( templateDim, templateDim ) );
            contour.push_back( Point( templateDim, templateDim >> 1 ) );
            contour.push_back( Point( templateDim >> 1, templateDim >> 1 ) );

            templ = Mat::zeros( Size( tempRotDim, tempRotDim ), CV_8UC1 );
            drawContours( templ( rect ), vector< vector< Point > >( 1, contour ), -1, Scalar( 56 ), FILLED );
            // imwrite( DEBUG_FOLDER + "templ_000.png", templ );

            templ( rect ) = mask( rect ) - templ( rect );
            // imwrite( DEBUG_FOLDER + "templ_001.png", templ );

            templ( rect ).setTo( 0, templ( rect ) > 200 );
            // imwrite( "/var/tmp/gaugecam/templ_002.png", templ );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::DrawCorner] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::RotateImage( const Mat &src, Mat &dst, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double angleAdj = 0.0 > angle ? 360.0 + angle : angle;
        Point2d ptCenter = Point2d( static_cast< double >( src.cols ) / 2.0, static_cast< double >( src.rows ) / 2.0 );
        Mat matRotMatrix = getRotationMatrix2D( ptCenter, angleAdj, 1.0 );
        warpAffine( src, dst, matRotMatrix, dst.size(), INTER_CUBIC );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// cornerIdx: 0 = top, left point, followint points move clockwise 1-7
GC_STATUS OctagonSearch::CreateTemplateOverlay( const std::string debugFolder )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( templates.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] Template vector empty";
            retVal = GC_ERR;
        }
        else if ( templates[ 0 ].ptTemplates[ 0 ].mask.empty() || templates[ 0 ].ptTemplates[ 0 ].templ.empty() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] mask and/or template empty";
            retVal = GC_ERR;
        }
        else
        {
#ifdef DEBUG_STOPSIGN_TEMPL
            imwrite( DEBUG_FOLDER + string( "center_template.png" ), templates[ 0 ].ptTemplates[ templates[ 0 ].ptTemplates.size() >> 1 ].templ );
            imwrite( DEBUG_FOLDER + string( "center_mask.png" ), templates[ 0 ].ptTemplates[ templates[ 0 ].ptTemplates.size() >> 1 ].mask );
#endif
            char buf[ 512 ];
            Mat scratch, tempColor = Mat::zeros( Size( templates[ 0 ].ptTemplates[ 0 ].mask.cols * 2,
                                                       templates[ 0 ].ptTemplates[ 0 ].mask.rows ), CV_8UC3 );
            for ( size_t j = 0; j < templates.size(); ++j )
            {
                for ( size_t i = 0; i < templates[ j ].ptTemplates.size(); ++i )
                {
                    cvtColor( templates[ j ].ptTemplates[ i ].templ, scratch, COLOR_GRAY2BGR );
                    scratch.copyTo( tempColor( Rect( 0, 0, templates[ j ].ptTemplates[ 0 ].mask.cols, templates[ j ].ptTemplates[ 0 ].mask.rows ) ) );
                    cvtColor( templates[ j ].ptTemplates[ i ].mask, scratch, COLOR_GRAY2BGR );
                    scratch.copyTo( tempColor( Rect( templates[ j ].ptTemplates[ 0 ].mask.cols, 0,
                                    templates[ j ].ptTemplates[ 0 ].mask.cols, templates[ j ].ptTemplates[ 0 ].mask.rows ) ) );

                    putText( tempColor, "Template", Point( 10, 20 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 255 ), 2 );
                    putText( tempColor, "Mask", Point( templates[ j ].ptTemplates[ 0 ].mask.cols + 10, 20 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 255 ), 2 );
                    sprintf( buf, "Angles pt=%3d templ=%+d", static_cast< int >( templates[ j ].pointAngle ),
                             cvRound( templates[ j ].ptTemplates[ i ].angle ) );
                    putText( tempColor, buf, Point( 10, 40 ), FONT_HERSHEY_PLAIN, 1.2, Scalar( 0, 255, 0 ), 1 );
                    sprintf( buf, "%stemplate%03d_%03d.png", debugFolder.c_str(),
                             static_cast< int >( templates[ j ].pointAngle ),
                             cvRound( templates[ j ].ptTemplates[ i ].angle ) + 5 );

                    line( tempColor, Point2d( templates[ j ].ptTemplates[ i ].offset.x - 10, templates[ j ].ptTemplates[ i ].offset.y ),
                          Point2d( templates[ j ].ptTemplates[ i ].offset.x + 10, templates[ j ].ptTemplates[ i ].offset.y ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y - 10 ),
                          Point2d( templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y + 10 ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x - 10, templates[ j ].ptTemplates[ i ].offset.y ),
                          Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x + 10, templates[ j ].ptTemplates[ i ].offset.y ), Scalar( 0, 0, 255 ), 1 );
                    line( tempColor, Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y - 10 ),
                          Point2d( templates[ j ].ptTemplates[ 0 ].mask.cols + templates[ j ].ptTemplates[ i ].offset.x, templates[ j ].ptTemplates[ i ].offset.y + 10 ), Scalar( 0, 0, 255 ), 1 );
                    imwrite( buf, tempColor );
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CreateTemplateOverlay] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::FitLine( const std::vector< Point > &pts, LineEnds &lineEnds, const cv::Mat &img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 5 > pts.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::FitLine] At least five points are needed to fit a line";
            retVal = GC_ERR;
        }
        else
        {
            double angle;
            Vec4d lineVec;

            fitLine( pts, lineVec, DIST_L2, 0.0, 0.01, 0.01 );
            lineEnds.top.x = cvRound( lineVec[ 2 ] + ( lineVec[ 0 ] * -lineVec[ 2 ] ) );
            lineEnds.top.y = cvRound( lineVec[ 3 ] + ( lineVec[ 1 ] * -lineVec[ 2 ] ) );
            lineEnds.bot.x = cvRound( lineVec[ 2 ] + ( lineVec[ 0 ] * ( img.cols - lineVec[ 2 ] - 1 ) ) );
            lineEnds.bot.y = cvRound( lineVec[ 3 ] + ( lineVec[ 1 ] * ( img.cols - lineVec[ 2 ] - 1 ) ) );
            angle = atan2( ( lineEnds.bot.y - lineEnds.top.y ),
                           ( lineEnds.bot.x - lineEnds.top.x ) ) * ( 180.0 / CV_PI );

            double rads = angle * CV_PI / 180.0;
            Point2d pt = Point2d( lineEnds.top.x + cos( rads ) * 100.0, lineEnds.top.y + sin( rads ) * 100 );

            bool isVertical;
            double slope, intercept;
            retVal = GetSlopeIntercept( lineEnds.top, pt, slope, intercept, isVertical );
            if ( GC_OK == retVal )
            {
                if ( isVertical )
                {
                    lineEnds.top.x = cvRound( pt.x );
                    lineEnds.top.y = 0;
                    lineEnds.bot.x = cvRound( pt.x );
                    lineEnds.bot.y = img.rows - 1;
                }
                else
                {
                    lineEnds.top.x = 0;
                    lineEnds.top.y = cvRound( intercept );
                    lineEnds.bot.x = img.cols - 1;
                    lineEnds.bot.y = cvRound( slope * lineEnds.bot.x + intercept );
                    clipLine( img.size(), lineEnds.top, lineEnds.bot );
                }
            }
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::FitLine] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::GetSlopeIntercept( const Point2d one, const Point2d two, double &slope, double &intercept, bool &isVertical )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( numeric_limits< double >::epsilon() > two.x - one.x )
        {
            isVertical = true;
            slope = numeric_limits< double >::max();
            intercept = -9999999;
        }
        else
        {
            isVertical = false;
            slope = ( two.y - one.y ) / ( two.x - one.x );
            intercept = one.y - slope * one.x;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::GetSlopeIntercept] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::GetRandomNumbers( const int low_bound, const int high_bound, const int cnt_to_generate,
                                            vector< int > &numbers, const bool isFirst )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( high_bound - low_bound + 1 < static_cast< int >( numbers.size() ) / 2 )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] Not enough points to find good numbers";
            retVal = GC_ERR;
        }
        else
        {
            default_random_engine randomEngine;
            if ( isFirst )
            {
                auto seed = std::chrono::system_clock::now().time_since_epoch().count(); //seed
                randomEngine.seed( static_cast< unsigned int >( seed ) );
            }

            std::uniform_int_distribution< int > di( low_bound, high_bound ); //distribution

            vector< int > tempVec;
            for ( int i = 0; i < cnt_to_generate; ++i )
                tempVec.push_back( 0 );

            bool foundIt;
            int tries = 10;
            numbers.clear();
            while ( tries > 0 && static_cast< int >( numbers.size() ) < cnt_to_generate )
            {
                std::generate( tempVec.begin(), tempVec.end(), [ & ]{ return di( randomEngine ); } );
                for ( size_t i = 0; i < tempVec.size(); ++i )
                {
                    foundIt = false;
                    for ( size_t j = 0; j < numbers.size(); ++j )
                    {
                        if ( tempVec[ i ] == numbers[ j ] )
                        {
                            foundIt = true;
                            break;
                        }
                    }
                    if ( !foundIt )
                    {
                        numbers.push_back( tempVec[ i ] );
                        if ( static_cast< int >( numbers.size() ) >= cnt_to_generate )
                            break;
                    }
                }
                --tries;
            }
            if ( static_cast< int >( numbers.size() ) < cnt_to_generate )
            {
                FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] Not enough unique numbers found";
                retVal = GC_ERR;
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::GetRandomNumbers] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctagonSearch::LineIntersection( const LineEnds line1, const LineEnds line2, cv::Point2d &r )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d x = Point2d( line2.top ) - Point2d( line1.top );
        Point2d d1 = Point2d( line1.bot ) - Point2d( line1.top );
        Point2d d2 = Point2d( line2.bot ) - Point2d( line2.top );

        double cross = d1.x * d2.y - d1.y * d2.x;
        if (abs(cross) < numeric_limits< double >::epsilon() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::LineIntersection] Lines are parallel";
            return GC_ERR;
        }

        double t1 = ( x.x * d2.y - x.y * d2.x ) / cross;
        r = Point2d( line1.top ) + d1 * t1;
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::LineIntersection] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctagonSearch::CalcPointsFromLines( const std::vector< LineEnds > lines, std::vector< cv::Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 8 != lines.size() )
        {
            FILE_LOG( logERROR ) << "[StopsignSearch::CalcPointsFromLines] Need 8 lines, but got only " << lines.size();
            retVal = GC_ERR;
        }
        else
        {
            pts.clear();
            Point2d point;
            for ( size_t i = 1; i < lines.size(); ++i )
            {
                retVal = LineIntersection( lines[ i - 1 ], lines[ i ], point );
                if ( GC_OK == retVal )
                {
                    pts.push_back( point );
                }
                else
                {
                    break;
                }
            }
            if ( GC_OK == retVal )
            {
                retVal = LineIntersection( lines[ lines.size() - 1 ], lines[ 0 ], point );
                if ( GC_OK == retVal )
                {
                    pts.push_back( point );
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[StopsignSearch::CalcPointsFromLines] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}


} // namespace gc
