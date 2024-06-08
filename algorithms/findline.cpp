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
#include "findline.h"
#include <iostream>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#ifdef DEBUG_FIND_LINE
#undef DEBUG_FIND_LINE
#include <iostream>
#include <filesystem>
#ifdef WIN32
static const std::string DEBUG_RESULT_FOLDER = "c:/gaugecam/";
#else
static const std::string DEBUG_RESULT_FOLDER = "/var/tmp/gaugecam/";
#endif
#endif

using namespace cv;
using namespace std;
#ifdef DEBUG_FIND_LINE
namespace fs = std::filesystem;
#endif

static const int MEDIAN_FILTER_KERN_SIZE = 9;

namespace gc
{

FindLine::FindLine() :
    m_minLineFindAngle( DEFAULT_MIN_LINE_ANGLE ),
    m_maxLineFindAngle( DEFAULT_MAX_LINE_ANGLE )
{
#ifdef DEBUG_FIND_LINE
    if ( !fs::exists( DEBUG_RESULT_FOLDER ) )
    {
        bool isOK = fs::create_directories( DEBUG_RESULT_FOLDER );
        if ( !isOK )
        {
            FILE_LOG( logERROR ) << "Could not create debug result folder: " << DEBUG_RESULT_FOLDER;
        }
    }
#endif
}

GC_STATUS FindLine::SetLineFindAngleBounds( const double minAngle, const double maxAngle )
{
    GC_STATUS retVal = GC_OK;
    if ( minAngle > maxAngle )
    {
        FILE_LOG( logERROR ) << "[FindLine::SetLineFindAngleBounds] Min angle must be less than max angle: min="
                             << minAngle << " max=" << maxAngle;
        retVal = GC_ERR;
    }
    else
    {
        m_minLineFindAngle = minAngle;
        m_maxLineFindAngle = maxAngle;
    }
    return retVal;
}
GC_STATUS FindLine::RemoveOutliers( vector< Point2d > &pts, const size_t numToKeep )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 7 > pts.size() )
        {
            FILE_LOG( logERROR ) << "[FindLine::RemoveOutliers] Point count to few to remove outliers";
            retVal = GC_ERR;
        }
        else if ( 5 > numToKeep || pts.size() <= numToKeep )
        {
            FILE_LOG( logERROR ) << "[FindLine::RemoveOutliers] Invalid number to keep in outlier removal";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > ptTemp = pts;
            sort( ptTemp.begin(), ptTemp.end(), []( Point2d a, Point2d b ) {
                return a.y > b.y; } );

            pts.clear();
            double medianY = ptTemp[ ptTemp.size() / 2 ].y;

            vector< Point3d > ptsDistFromMedian;
            for ( size_t i = 0; i < ptTemp.size(); ++i )
            {
                ptsDistFromMedian.push_back( Point3d( ptTemp[ i ].x, ptTemp[ i ].y, fabs( medianY - ptTemp[ i ].y ) ) );
            }
            sort( ptsDistFromMedian.begin(), ptsDistFromMedian.end(), []( Point3d a, Point3d b ) {
                return a.z < b.z; } );

            for ( size_t i = 0; i < numToKeep; ++i )
            {
                pts.push_back( Point2d( ptsDistFromMedian[ i ].x, ptsDistFromMedian[ i ].y ) );
            }

            sort( pts.begin(), pts.end(), []( Point2d a, Point2d b ) {
                return a.x > b.x; } );
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[FindLine::RemoveOutliers] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindLine::TriagePoints( vector< Point2d > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 7 > pts.size() )
        {
            FILE_LOG( logERROR ) << "[FindLine::TriagePoints] Point count to few to triage";
            retVal = GC_ERR;
        }
        else
        {
            vector< Point2d > ptTemp = pts;
            sort( ptTemp.begin(), ptTemp.end(), []( Point2d a, Point2d b ) {
                return a.y > b.y; } );

            pts.clear();
            double medianY = ptTemp[ ptTemp.size() / 2 ].y;
            sort( ptTemp.begin(), ptTemp.end(), []( Point2d a, Point2d b ) {
                return a.x > b.x; } );
            for ( size_t i = 0; i < ptTemp.size(); ++i )
            {
                if ( 17.0 > fabs( medianY - ptTemp[ i ].y ) )
                {
                     pts.push_back( ptTemp[ i ] );
                }
            }
            if ( pts.size() < 5 )
            {
                FILE_LOG( logERROR ) << "[FindLine::TriagePoints] Points do not form a line";
                retVal = GC_ERR;
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[FindLine::TriagePoints] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindLine::Find( const Mat &img, const vector< LineEnds > &lines, FindLineResult &result )
{
    result.findSuccess = false;
    GC_STATUS retVal = GC_OK;
    if ( lines.empty() || img.empty() )
    {
        FILE_LOG( logERROR ) << "[FindLine::Find] Cannot find lines with no search lines defined or in a NULL image";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
#ifdef DEBUG_FIND_LINE
            Mat outImg;
            if ( CV_8UC1 == img.type() )
                cvtColor( img, outImg, COLOR_GRAY2BGR );
            else if ( CV_8UC3 == img.type() )
                outImg = img.clone();
            else
            {
                FILE_LOG( logERROR ) << "[FindLine::Find] Invalid image type for drawing row sum must be 8-bit gray or 8-bit bgr";
                retVal = GC_ERR;
            }
#endif

            Mat inImg;
            if ( CV_8UC3 == img.type() )
                cvtColor( img, inImg, COLOR_BGR2GRAY );
            else if ( CV_8UC1 == img.type() )
                inImg = img.clone();
            else
            {
                FILE_LOG( logERROR ) << "[FindLine::Find] Invalid image type for find. Must be 8-bit gray or 8-bit bgr";
                retVal = GC_ERR;
            }

            if ( GC_OK == retVal )
            {
                // clean-up a little
                Mat scratch;
                retVal = Preprocess( inImg, scratch );
                if ( GC_OK == retVal )
                {
#ifdef DEBUG_FIND_LINE
                    bool isOK = imwrite( DEBUG_RESULT_FOLDER + "preprocess.png", scratch );
#endif
                    size_t start;
                    Point2d linePt;
                    vector< uint > rowSums;
                    string timestamp = result.timestamp;
                    result.timestamp = timestamp;
                    size_t linesPerSwath = lines.size() / 10;
                    for ( size_t i = 0; i < 9; ++i )
                    {
                        start = i * linesPerSwath;
                        retVal = EvaluateSwath( scratch, lines, start, start + linesPerSwath, linePt, result );
                        if ( GC_OK == retVal )
                            result.foundPoints.push_back( linePt );
                    }
                    start = lines.size() - linesPerSwath - 1;
                    retVal = EvaluateSwath( scratch, lines, start, lines.size() - 1, linePt, result );
                    if ( GC_OK == retVal )
                        result.foundPoints.push_back( linePt );

#ifdef DEBUG_FIND_LINE
                    line( outImg, lines[ 0 ].top, lines[ 0 ].bot, Scalar( 0, 255, 255 ), 3 );
                    line( outImg, lines[ lines.size() - 1 ].top, lines[ lines.size() - 1 ].bot, Scalar( 0, 255, 255 ), 3 );
                    isOK = imwrite( DEBUG_RESULT_FOLDER + "rowsums.png", outImg );
                    if ( !isOK )
                    {
                        FILE_LOG( logERROR ) << "[FindLine::Find] Could not write debug image " << DEBUG_RESULT_FOLDER << "rowsums.png";
                    }
#endif

                    double xCenter = ( lines[ 0 ].bot.x + lines[ lines.size() - 1 ].bot.x ) / 2.0;
                    retVal = TriagePoints( result.foundPoints );
                    if ( GC_OK == retVal )
                    {
                        retVal = FitLineRANSAC( result.foundPoints, result.calcLinePts, xCenter, scratch );
                        if ( GC_OK == retVal )
                        {
                            result.findSuccess = true;
                        }
                    }
                    if ( GC_OK != retVal )
                    {
                        retVal = RemoveOutliers( result.foundPoints, 5 );
                        if ( GC_OK == retVal )
                        {
                            retVal = FitLineRANSAC( result.foundPoints, result.calcLinePts, xCenter, scratch );
                            if ( GC_OK == retVal )
                            {
                                result.findSuccess = true;
                            }
                        }
                    }
                }
            }
        }
        catch( cv::Exception &e )
        {
            result.findSuccess = false;
            FILE_LOG( logERROR ) << "[FindLine::Find] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS FindLine::Preprocess( const cv::Mat &src, cv::Mat &dst )
{
    GC_STATUS retVal = GC_OK;
    if ( src.empty() )
    {
        FILE_LOG( logERROR ) << "[FindLine::Preprocess] Not possible to preprocess an empty image";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            GaussianBlur( src, dst, Size( 11, 11 ), 3.0 );
            medianBlur( dst, dst, 23 );

            Mat kern = getStructuringElement( MORPH_RECT, Size( 5, 11 ) );
            dilate( dst, dst, kern, Point( -1, -1 ), 2 );
            erode( dst, dst, kern, Point( -1, -1 ), 2 );
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::Preprocess] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS FindLine::FitLineRANSAC( const std::vector< Point2d > &pts, FindPointSet &findPtSet,
                                   const double xCenter, const cv::Mat &img )
{
    GC_STATUS retVal = 5 > pts.size() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::FitLineRANSAC] At least five points are needed to fit a line";
    }
    else
    {
        try
        {
#ifdef DEBUG_FIND_LINE
            Mat scratch;
            if ( CV_8UC1 == img.type() )
                cvtColor( img, scratch, COLOR_GRAY2BGR );
            else
                scratch = img.clone();
#endif
            Vec4d lineVec;
            vector< int > indices;
            vector< Point2d > ptSet;
            vector< FindPointSet > validLines;
            for ( int i = 0; i < FIT_LINE_RANSAC_TRIES_TOTAL; ++i )
            {
                retVal = GetRandomNumbers( 0, static_cast< int >( pts.size() ) - 1, FIT_LINE_RANSAC_POINT_COUNT, indices, 0 == i );
                if ( GC_OK == retVal )
                {
                    ptSet.clear();
                    for ( size_t i = 0; i < indices.size(); ++i )
                    {
#ifdef DEBUG_FIND_LINE
                        circle( scratch, pts[ indices[ i ] ], 5, Scalar( 0, 255, 255 ), 3 );
#endif
                        ptSet.push_back( pts[ indices[ i ] ] );
                    }

                    fitLine( ptSet, lineVec, DIST_L2, 0.0, 0.01, 0.01 );
                    findPtSet.lftPixel.x = lineVec[ 2 ] + ( lineVec[ 0 ] * -lineVec[ 2 ] );
                    findPtSet.lftPixel.y = lineVec[ 3 ] + ( lineVec[ 1 ] * -lineVec[ 2 ] );
                    findPtSet.rgtPixel.x = lineVec[ 2 ] + ( lineVec[ 0 ] * ( img.cols - lineVec[ 2 ] - 1 ) );
                    findPtSet.rgtPixel.y = lineVec[ 3 ] + ( lineVec[ 1 ] * ( img.cols - lineVec[ 2 ] - 1 ) );
                    findPtSet.ctrPixel.x = lineVec[ 2 ] + ( lineVec[ 0 ] * ( xCenter - lineVec[ 2 ] ) );
                    findPtSet.ctrPixel.y = lineVec[ 3 ] + ( lineVec[ 1 ] * ( xCenter - lineVec[ 2 ] ) );
                    findPtSet.anglePixel = atan2( ( findPtSet.rgtPixel.y - findPtSet.lftPixel.y ),
                                                 ( findPtSet.rgtPixel.x - findPtSet.lftPixel.x ) ) * ( 180.0 / CV_PI );
#ifdef DEBUG_FIND_LINE
                    line( scratch, findPtSet.lftPixel, findPtSet.rgtPixel, Scalar( 0, 0, 255 ), 1 );
#endif
                    if ( m_minLineFindAngle <= findPtSet.anglePixel && m_maxLineFindAngle >= findPtSet.anglePixel )
                    {
                        validLines.push_back( findPtSet );
                    }
                    if ( validLines.size() >= FIT_LINE_RANSAC_TRIES_EARLY_OUT )
                        break;
                }
            }
#ifdef DEBUG_FIND_LINE
            imwrite( DEBUG_RESULT_FOLDER + "ransac.png", scratch );
#endif
            if ( 9 > validLines.size() )
            {
                FILE_LOG( logERROR ) << "[FindLine::FitLineRANSAC] No valid lines found";
                retVal = GC_ERR;
            }
            else
            {
                sort( validLines.begin(), validLines.end(), []( FindPointSet a, FindPointSet b ) {
                    return a.ctrPixel.y > b.ctrPixel.y; } );

                double totalY = 0.0;
                double totalTheta = 0.0;
                size_t start = validLines.size() >> 2;
                size_t end = validLines.size() - start;
                for ( size_t i = start; i < end; ++i )
                {
                    totalY += validLines[ i ].ctrPixel.y;
                    totalTheta += validLines[ i ].anglePixel;
                }
                findPtSet.ctrPixel.x = xCenter;
                findPtSet.ctrPixel.y = totalY / static_cast< double >( end - start );
                findPtSet.anglePixel = totalTheta / static_cast< double >( end - start );

                double rads = findPtSet.anglePixel * CV_PI / 180.0;
                Point2d pt = Point2d( findPtSet.ctrPixel.x + cos( rads ) * 100.0, findPtSet.ctrPixel.y + sin( rads ) * 100 );

                double slope, intercept;
                retVal = GetSlopeIntercept( findPtSet.ctrPixel, pt, slope, intercept );
                if ( GC_OK == retVal )
                {
                    findPtSet.lftPixel.x = 0.0;
                    findPtSet.lftPixel.y = intercept;
                    findPtSet.rgtPixel.x = static_cast< double >( img.cols ) - 1.0;
                    findPtSet.rgtPixel.y = slope * findPtSet.rgtPixel.x + intercept;
                }
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::FitLineRANSAC] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindLine::GetSlopeIntercept( const Point2d one, const Point2d two, double &slope, double &intercept )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        slope = ( two.y - one.y ) / ( 0.0 == ( two.x - one.x ) ? std::numeric_limits< double >::epsilon() : ( two.x - one.x ) );
        intercept = one.y - slope * one.x;
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[FindLine::GetSlopeIntercept] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindLine::GetRandomNumbers( const int low_bound, const int high_bound, const int cnt_to_generate,
                                      vector< int > &numbers, const bool isFirst )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( high_bound - low_bound + 1 < static_cast< int >( numbers.size() ) / 2 )
        {
            FILE_LOG( logERROR ) << "[FindLine::GetRandomNumbers] Not enough points to find good numbers";
            retVal = GC_ERR;
        }
        else
        {
            if ( isFirst )
            {
                auto seed = std::chrono::system_clock::now().time_since_epoch().count(); //seed
                m_randomEngine.seed( static_cast< unsigned int >( seed ) );
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
                std::generate( tempVec.begin(), tempVec.end(), [ & ]{ return di( m_randomEngine ); } );
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
                FILE_LOG( logERROR ) << "[FindLine::GetRandomNumbers] Not enough unique numbers found";
                retVal = GC_ERR;
            }

//#ifdef DEBUG_FIND_LINE
//            for ( size_t i = 0; i < static_cast< size_t >( cnt_to_generate ); ++i )
//                std::cout << numbers[ i ] << ", ";
//            std::cout << endl;
//#endif
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[FindLine::GetRandomNumbers] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindLine::DrawResult( const Mat &img, Mat &imgOut, const FindLineResult &result,
                                const IMG_DISPLAY_OVERLAYS overlayTypes )
{
    GC_STATUS retVal = GC_OK;
    if ( ( img.ptr() == imgOut.ptr() ) || img.empty() )
    {
        FILE_LOG( logERROR ) << "[FindLine::DrawResult] Cannot draw find line results on a NULL image or if src=dst";
        retVal = GC_ERR;
    }
    else
    {
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
                FILE_LOG( logERROR ) << "[FindLine::DrawResult] Invalid image type for drawing row sum must be 8-bit gray or 8-bit bgr";
                retVal = GC_ERR;
            }

            if ( GC_OK == retVal )
            {
                int circleSize =  std::max( 5, cvRound( static_cast< double >( imgOut.rows ) / 400.0 ) );
                int textStroke = std::max( 1, cvRound( static_cast< double >( imgOut.rows ) / 300.0 ) );
                int textRowSpacing = cvRound( static_cast< double >( imgOut.rows ) / 40.0 );
                double fontScale = static_cast< double >( imgOut.rows ) / 500.0;

                if ( ( overlayTypes & DIAG_ROWSUMS ) && !result.diagRowSums.empty() )
                {
                    for ( size_t i = 0; i < result.diagRowSums.size(); ++i )
                    {
                        if ( result.diagRowSums[ i ].size() > 1 )
                        {
                            for ( size_t j = 1; j < result.diagRowSums[ i ].size(); ++j )
                            {
                                line( imgOut, result.diagRowSums[ i ][ j - 1 ], result.diagRowSums[ i ][ j ], Scalar( 0, 255, 255 ), 2 );
                            }
                        }
                    }
                }

                if ( ( overlayTypes & FINDLINE_1ST_DERIV ) && !result.diag1stDeriv.empty() )
                {
                    for ( size_t i = 0; i < result.diag1stDeriv.size(); ++i )
                    {
                        if ( result.diag1stDeriv[ i ].size() > 1 )
                        {
                            for ( size_t j = 1; j < result.diag1stDeriv[ i ].size(); ++j )
                            {
                                line( imgOut, result.diag1stDeriv[ i ][ j - 1 ], result.diag1stDeriv[ i ][ j ], Scalar( 0, 0, 255 ), 2 );
                            }
                        }
                    }
                }

                if ( ( overlayTypes & FINDLINE_2ND_DERIV ) && !result.diag2ndDeriv.empty() )
                {
                    for ( size_t i = 0; i < result.diag2ndDeriv.size(); ++i )
                    {
                        if ( result.diag2ndDeriv[ i ].size() > 1 )
                        {
                            for ( size_t j = 1; j < result.diag2ndDeriv[ i ].size(); ++j )
                            {
                                line( imgOut, result.diag2ndDeriv[ i ][ j - 1 ], result.diag2ndDeriv[ i ][ j ], Scalar( 255, 127, 127 ), 2 );
                            }
                        }
                    }
                }

                if ( !result.findSuccess )
                {
                    line( imgOut, Point2d( 0.0, 0.0 ), Point2d( img.cols - 1, img.rows - 1 ), Scalar( 0, 0, 255 ), 3 );
                    line( imgOut, Point2d( 0.0, img.rows - 1 ), Point2d( img.cols - 1, 0.0 ), Scalar( 0, 0, 255 ), 3 );
                    putText( imgOut, "BAD FIND", Point( 5, textRowSpacing * 2 ), FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 0, 255 ), textStroke );
                    for ( size_t i = 0; i < result.foundPoints.size(); ++i )
                    {
                        circle( imgOut, result.foundPoints[ i ], max( 3, circleSize >> 1 ), Scalar( 0, 255, 255 ), FILLED );
                    }
                }
                else
                {
                    if ( ( overlayTypes & FINDLINE ) )
                    {
                        line( imgOut, result.calcLinePts.lftPixel, result.calcLinePts.rgtPixel, Scalar( 255, 0, 0 ), textStroke + 1 );
                        circle( imgOut, result.calcLinePts.ctrPixel, circleSize + textStroke, Scalar( 0, 255, 0 ), textStroke );
                        line( imgOut, Point2d( result.calcLinePts.ctrPixel.x - circleSize - textStroke * 2,
                                               result.calcLinePts.ctrPixel.y - circleSize - textStroke * 2 ),
                                      Point2d( result.calcLinePts.ctrPixel.x + circleSize + textStroke * 2,
                                               result.calcLinePts.ctrPixel.y + circleSize + textStroke * 2 ), Scalar( 0, 0, 255 ), textStroke );
                        line( imgOut, Point2d( result.calcLinePts.ctrPixel.x + circleSize + textStroke * 2,
                                               result.calcLinePts.ctrPixel.y - circleSize - textStroke * 2 ),
                                      Point2d( result.calcLinePts.ctrPixel.x - circleSize - textStroke * 2,
                                               result.calcLinePts.ctrPixel.y + circleSize + textStroke * 2 ), Scalar( 0, 0, 255 ), textStroke );
                    }

                    if ( ( overlayTypes & MOVE_FIND ) )
                    {
                        line( imgOut, result.refMovePts.lftPixel, result.refMovePts.rgtPixel, Scalar( 0, 0, 255 ), circleSize );
                        line( imgOut, result.foundMovePts.lftPixel, result.foundMovePts.rgtPixel, Scalar( 0, 255, 0 ), ( circleSize >> 1 ) - 1 );
                    }
                }
                if ( 3 < result.foundPoints.size() && ( overlayTypes & RANSAC_POINTS ) )
                {
                    for ( size_t i = 0; i < result.foundPoints.size(); ++i )
                    {
                        circle( imgOut, result.foundPoints[ i ], max( 3, circleSize >> 1 ), Scalar( 0, 255, 255 ), FILLED );
                    }
                }
                for ( size_t i = 0; i < result.msgs.size(); ++i )
                {
                    putText( imgOut, result.msgs[ i ], Point( 3, ( static_cast< int >( i ) + 1 ) * textRowSpacing + 50 ), FONT_HERSHEY_PLAIN, fontScale, Scalar( 0, 255, 255 ), textStroke );
                }
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::DrawResult] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindLine::EvaluateSwath( const Mat &img, const vector< LineEnds > &lines, const size_t startIndex,
                                   const size_t endIndex, Point2d &resultPt, FindLineResult &result )
{
    GC_STATUS retVal = ( lines.empty() || img.empty() || startIndex > endIndex ||
                         lines.size() - 1 < endIndex ) ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::EvaluateSwath] Cannot evalate swath with invalid indices or an empty line vector or image";
    }
    else
    {
        try
        {
            vector< LineEnds > swath;
            for ( size_t i = startIndex; i <= endIndex; ++i )
                swath.push_back( lines[ i ] );

            vector< uint > rowSums;
            retVal = CalcRowSums( img, swath, rowSums );
            if ( GC_OK == retVal )
            {
                retVal = CalculateRowSumsLines( rowSums, swath, result.diagRowSums,
                                                result.diag1stDeriv, result.diag2ndDeriv );
                if ( GC_OK != retVal )
                {
                    FILE_LOG( logWARNING ) << "[FindLine::EvaluateSwath] Cannot retrieve diagnostic line points";
                    retVal = GC_OK;
                }
                retVal = CalcSwathPoint( swath, rowSums, resultPt );
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::EvaluateSwath] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS FindLine::CalcSwathPoint( const vector< LineEnds > &swath, const vector< uint > &rowSums, Point2d &resultPt )
{
    GC_STATUS retVal = ( swath.empty() || rowSums.empty() ) ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::CalcSwathPoint] Cannot calculate swath point with empty line or rowsums vector(s)";
    }
    else
    {
        try
        {
            int diff ;
            int index = -1;
            int diffMax = numeric_limits< int >::min();
            for ( size_t i = 1; i < rowSums.size() - 1; ++i )
            {
                diff = abs( static_cast< int >( rowSums[ i ] ) - static_cast< int >( rowSums[ i - 1 ] ) );
                if ( diff > diffMax )
                {
                    index = static_cast< int >( i );
                    diffMax = diff;
                }
            }
            if ( -1 == index )
            {
                FILE_LOG( logERROR ) << "[FindLine::CalcSwathPoint] No swath edge found";
                retVal = GC_WARN;
            }
            else
            {
                resultPt.x = static_cast< double >( swath[ 0 ].top.x + swath[ swath.size() - 1 ].top.x ) / 2.0;
                double d0 = rowSums[ static_cast< size_t >( index - 1 ) ] - rowSums[ static_cast< size_t >( index - 2 ) ];
                double d1 = rowSums[ static_cast< size_t >( index ) ] - rowSums[ static_cast< size_t >( index - 1 ) ];
                double d2 = rowSums[ static_cast< size_t >( index + 1 ) ] - rowSums[ static_cast< size_t >( index ) ];
                double dd1 = d1 - d0;
                double dd2 = d2 - d1;
                resultPt.y = static_cast< double >( index ) + fabs( dd1 ) / ( fabs( dd1 * dd1 ) + ( dd2 * dd2 ) ) +
                        static_cast< double >( swath[ 0 ].top.y + swath[ swath.size() - 1 ].top.y ) / 2.0;
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::CalcSwathPoint] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindLine::CalcRowSums( const Mat &img, const vector< LineEnds > &lines, vector< uint > &rowSums )
{
    GC_STATUS retVal = lines.empty() || img.empty() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::CalcRowSums] Cannot calculate row sums with no search lines defined or in a NULL image";
    }
    else
    {
        try
        {
            Mat color;
            cvtColor( img, color, COLOR_GRAY2BGR );
            line( color, lines[ 0 ].top, lines[ 0 ].bot, Scalar( 0, 0, 255 ), 3 );
            line( color, lines[ lines.size() - 1 ].top, lines[ lines.size() - 1 ].bot, Scalar( 0, 0, 255 ), 3 );
            // imwrite( "/var/tmp/water/orig.png", color );


            rowSums.clear();
            vector< uint > rowSumsTemp;
            int height = lines[ 0 ].bot.y - lines[ 0 ].top.y;
            for ( int i = 0; i < height; ++i )
                rowSumsTemp.push_back( 0 );

            for ( size_t i = 0; i < lines.size(); ++i )
            {
                LineIterator iter( img, lines[ i ].top, lines[ i ].bot );
                for ( int j = 0; j < std::min( height, iter.count ); ++j, ++iter )
                {
                    rowSumsTemp[ static_cast< size_t >( j ) ] += static_cast< uint >( **iter );
                }
            }
            retVal = MedianFilter( MEDIAN_FILTER_KERN_SIZE, rowSumsTemp, rowSums );
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::CalcRowSums] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS FindLine::CalculateRowSumsLines( const vector< uint > rowSums, const vector< LineEnds > lines, vector< vector< Point > > &rowSumsLines,
                                           vector< vector< Point > > &deriveOneLines,  vector< vector< Point > > &deriveTwoLines )
{
    GC_STATUS retVal = lines.empty() || rowSums.empty() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::CalculateRowSumsLines] Cannot calculate row sums line points with empty search lines, image, or row sum array";
    }
    else
    {
        try
        {
            uint maxVal = 0;
            for ( size_t i = 0; i < rowSums.size(); ++i )
            {
                if ( maxVal < rowSums[ i ] )
                    maxVal = rowSums[ i ];
            }

            int beg = min( lines[ 0 ].top.x, lines[ 0 ].bot.x ); // + width_poly;
            // double wide = static_cast< double >( min( lines[ lines.size() - 1 ].top.x, lines[ lines.size() - 1 ].bot.x ) - beg - 3 );
            double wide = 64.0;

            int right;
            int y = lines[ 0 ].top.y;
            double dMaxVal = static_cast< double >( maxVal );

            vector< Point > rowSumsPts;
            right = beg + cvRound( wide * ( static_cast< double >( rowSums[ 0 ] ) / dMaxVal ) );
            Point pt = Point( right, y );
            rowSumsPts.push_back( pt );
            ++y;

            int diff;
            int minDiff = 9999999;
            int maxDiff = -9999999;
            vector< int > firstDeriv;
            for ( size_t i = 1; i < rowSums.size(); ++i )
            {
                diff = rowSums[ i ] - rowSums[ i - 1 ];
                right = beg + cvRound( wide * ( static_cast< double >( rowSums[ i ] ) / dMaxVal ) );
                pt = Point( right, y );
                rowSumsPts.push_back( pt );
                ++y;

                firstDeriv.push_back( diff );
                if ( diff > maxDiff )
                    maxDiff = diff;
                if ( diff < minDiff )
                    minDiff = diff;
            }

            rowSumsLines.push_back( rowSumsPts );

            if ( maxDiff == minDiff )
            {
                FILE_LOG( logERROR ) << "[FindLine::CalculateRowSumsLines] Cannot calculate row sums first deriv if all values are the same";
                retVal = GC_ERR;
            }
            else
            {
                beg += 35;
                y = lines[ 0 ].top.y + 1;
                rowSumsPts.clear();

                int minDiff2 = 9999999;
                int maxDiff2 = -9999999;
                vector< int > secondDeriv;
                double totDiff = maxDiff - minDiff;
                right = beg + cvRound( wide * ( static_cast< double >( firstDeriv[ 0 ] - minDiff ) / totDiff ) );
                pt = Point( right, y );
                rowSumsPts.push_back( pt );
                ++y;

                for ( size_t i = 1; i < firstDeriv.size(); ++i )
                {
                    diff = firstDeriv[ i ] - firstDeriv[ i - 1 ];
                    right = beg + cvRound( wide * ( static_cast< double >( firstDeriv[ i ] - minDiff ) / totDiff ) );
                    pt = Point( right, y );
                    rowSumsPts.push_back( pt );
                    ++y;

                    secondDeriv.push_back( diff );
                    if ( diff > maxDiff2 )
                        maxDiff2 = diff;
                    if ( diff < minDiff2 )
                        minDiff2 = diff;
                }

                deriveOneLines.push_back( rowSumsPts );

                if ( maxDiff2 == minDiff2 )
                {
                    FILE_LOG( logERROR ) << "[FindLine::CalculateRowSumsLines] Cannot calculate row sums second deriv if all values are the same";
                    retVal = GC_ERR;
                }
                else
                {
                    beg += 70;
                    y = lines[ 0 ].top.y + 1;
                    rowSumsPts.clear();

                    double totDiff = maxDiff2 - minDiff;
                    right = beg + cvRound( wide * ( static_cast< double >( secondDeriv[ 0 ] - minDiff2 ) / totDiff ) );
                    pt = Point( right, y );
                    rowSumsPts.push_back( pt );
                    ++y;

                    for ( size_t i = 1; i < secondDeriv.size(); ++i )
                    {
                        right = beg + cvRound( wide * ( static_cast< double >( secondDeriv[ i ] - minDiff2 ) / totDiff ) );
                        pt = Point( right, y );
                        rowSumsPts.push_back( pt );
                        ++y;
                    }

                    deriveTwoLines.push_back( rowSumsPts );
                }
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::CalculateRowSumsLines] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS FindLine::MedianFilter( const size_t kernSize, const vector< uint > values, vector< uint > &valuesOut )
{
    GC_STATUS retVal = values.empty() || 3 > kernSize || kernSize * 2 > values.size() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[FindLine::MedianFilter] Median filter not possible with empty vector or bad kern size=" << kernSize;
    }
    else
    {
        try
        {
            valuesOut.clear();
            vector< uint > sortVec;
            size_t halfVec, kernHalf = kernSize >> 1;
            for ( size_t i = 0; i < kernHalf; ++i )
            {
                sortVec.clear();
                for ( size_t j = 0; j < kernHalf + i; j++ )
                    sortVec.push_back( values[ j ] );
                halfVec = sortVec.size() / 2;
                nth_element( sortVec.begin(), sortVec.begin() + static_cast< int >( halfVec ), sortVec.end() );
                if ( 0 == sortVec.size() % 2 )
                    valuesOut.push_back( ( sortVec[ halfVec ] + sortVec[ halfVec + 1 ] ) / 2 );
                else
                    valuesOut.push_back( sortVec[ halfVec ] );
            }
            for ( int i = static_cast< int >( kernHalf ); i < static_cast< int >( values.size() - kernHalf ); ++i )
            {
                sortVec.clear();
                for ( int j = -static_cast< int >( kernHalf ); j < static_cast< int >( kernHalf ); j++ )
                    sortVec.push_back( values[ static_cast< size_t >( i + j ) ] );
                nth_element( sortVec.begin(), sortVec.begin() + static_cast< int >( kernHalf ), sortVec.end() );
                valuesOut.push_back( sortVec[ kernHalf ] );
            }
            for ( size_t i = values.size() - kernHalf; i < values.size(); ++i )
            {
                sortVec.clear();
                for ( size_t j = i - kernHalf; j < values.size(); j++ )
                    sortVec.push_back( values[ j ] );
                halfVec = sortVec.size() / 2;
                nth_element( sortVec.begin(), sortVec.begin() + static_cast< int >( halfVec ), sortVec.end() );
                if ( 0 == sortVec.size() % 2 )
                    valuesOut.push_back( ( sortVec[ halfVec ] + sortVec[ halfVec + 1 ] ) / 2 );
                else
                    valuesOut.push_back( sortVec[ halfVec ] );
            }
        }
        catch( cv::Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindLine::MedianFilter] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
} // namespace gc
