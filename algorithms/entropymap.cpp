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

#include "entropymap.h"
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

#ifdef DEBUG_ENTROPY
#undef DEBUG_ENTROPY
#include <opencv2/imgcodecs.hpp>
#ifdef WIN32
#define DEBUG_FOLDER (std::string("C:/junk/water/"))
#else
#define DEBUG_FOLDER (std::string("/var/tmp/water/"))
#endif
#endif

#ifndef DO_TIME_TEST
#define DO_TIME_TEST
#include <chrono>
#endif

using namespace cv;
using namespace std;
using namespace boost;

static const int ENTROPY_MAP_KERNEL_SIZE_MIN = 3;
static const int ENTROPY_MAP_KERNEL_SIZE_MAX = 1024;

#ifdef DO_TIME_TEST
string GetNowString()
{
    chrono::system_clock::time_point now = chrono::system_clock::now();
    time_t now_c = chrono::system_clock::to_time_t( now );
    tm now_tm = *std::localtime( &now_c );
    char timeStamp[ 64 ];
    strftime( timeStamp, 64, "%Y-%m-%dT%H:%M:%S", &now_tm );
    return string( timeStamp );
}
#endif

namespace gc
{

EntropyMap::EntropyMap()
{
}
EntropyMap::~EntropyMap()
{
}
GC_STATUS EntropyMap::CalcMap( const Mat &src, Mat &dst, const int kernelSize, const bool useEllipse )
{
    GC_STATUS retVal = src.empty() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcMap] Cannot calculate entropy map for empty image";
        retVal = GC_ERR;
    }
    else
    {
        if ( 3 > kernelSize )
        {
            FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcMap] Cannot calculate entropy map with kernSize less than 3";
            retVal = GC_ERR;
        }
        else
        {
            try
            {
#ifdef DEBUT_ENTROPY
                imwrite( DEBUG_FOLDER + string("__gray_src.png"), src );
#endif
                Mat ellipseMask = Mat::zeros( kernelSize, kernelSize, CV_8UC1 );
                if ( useEllipse )
                {
                    int kernHalf = kernelSize >> 1;
                    ellipse( ellipseMask, Point( kernHalf, kernHalf ), Size( kernelSize, kernelSize ),
                             0.0, 0.0, 360.0, Scalar( 255 ), FILLED );
                }
                else
                {
                    ellipseMask.setTo( 255 );
                }
                Mat scratch = Mat::zeros( src.size(), CV_32FC1 );
                retVal = CalcTile( src, scratch, kernelSize, ellipseMask );
                if ( GC_OK == retVal )
#if 1
                    scratch.copyTo( dst );
#else
                    scratch( Rect( 0, 0, scratch.cols >> 2, scratch.rows >> 2 ) ).copyTo( dst );
#endif

#ifdef DEBUT_ENTROPY
                imwrite( DEBUG_FOLDER + string("__morph_8u.png"), dst );
#endif
            }
            catch( Exception &e )
            {
                FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcMap] " << e.what();
                retVal = GC_EXCEPT;
            }
        }
    }
    return retVal;
}
GC_STATUS EntropyMap::CalcTile( const Mat &src, Mat &dst, const int kernelSize, const cv::Mat &mask )
{
    GC_STATUS retVal = src.empty() ? GC_ERR : GC_OK;
    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcTile] Cannot calculate entropy map for empty image";
        retVal = GC_ERR;
    }
    else
    {
        if ( ENTROPY_MAP_KERNEL_SIZE_MIN > kernelSize ||
             ENTROPY_MAP_KERNEL_SIZE_MAX < kernelSize )
        {
            FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcTile] Invalid entropy map with kernSize=" << kernelSize \
                                 << ". Must be in range " << ENTROPY_MAP_KERNEL_SIZE_MIN << " to " << ENTROPY_MAP_KERNEL_SIZE_MAX << ".";
            retVal = GC_ERR;
        }
        try
        {
            int kernHalf = std::min( std::min( src.rows, src.cols ), kernelSize ) >> 1;

            if ( !mask.empty() )
            {
#ifdef DEBUT_ENTROPY
                imwrite(DEBUG_FOLDER + string("quarter_src.png"), src );
#endif
                float *pPix32f;
                for ( int row = kernHalf, centerRow = 0, dstRow = kernHalf >> 2;
                      row < src.rows - kernHalf; row += 4, centerRow += 4, ++dstRow )
                {
                    pPix32f = reinterpret_cast< float * >( dst.ptr( dstRow ) );
                    for ( int col = kernHalf, centerCol = 0, dstCol = kernHalf >> 2;
                          col < src.cols - kernHalf; col += 4, centerCol += 4, ++dstCol )
                    {
                        retVal = CalcEntropyValue( src( Rect( centerCol, centerRow, kernelSize, kernelSize ) ), pPix32f[ dstCol ], mask );
                    }
                }
#ifdef DEBUT_ENTROPY
                imwrite( DEBUG_FOLDER + "quarter_dst.png", dst );
#endif
            }
            else
            {
                FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcTile] Has no mask";
            }
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::CalcTile] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS EntropyMap::CalcEntropyValue( const Mat &img, float &entropy, const Mat &mask )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        size_t histSize = 32;
        // apply the mask
        Mat imgLocal;
        img.copyTo( imgLocal, mask );

        // do simpler special hist
        std::vector< int > bins( histSize, 0 );// close to zero but finite
        unsigned char * pRow;
        for (int row = 0; row < imgLocal.rows; row++)
        {
            pRow = reinterpret_cast< unsigned char * >( imgLocal.ptr( row ) );
            for ( int col = 0; col < imgLocal.cols; col++ )
            {
                ++bins[ pRow[ col ] >> 3 ];    // [0...255] --> [0...63]
            }
        }
        int count = 0;
        for ( size_t i = 0; i < histSize; ++i )
        {
            if ( bins[ i ] )
                ++count;
        }

//        entropy = log( static_cast< float > ( count ) );
        entropy = static_cast< float >( count );
    }
    catch (Exception &e)
    {
        FILE_LOG(logERROR) << "[" << __func__ << "][EntropyMap::CalcEntropyValue] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS EntropyMap::BuildLogLUT( const Mat &mask, vector< float > &lut )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        int count = cvRound( sum( mask ).val[ 0 ] / 255.0 );

        lut.clear();
        lut.push_back( 0.0f );
        for ( int i = 1; i < count; ++i )
            lut.push_back( static_cast< float >( log( static_cast< double >( i ) / static_cast< double >( count ) ) ) );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[" << __func__ << "][EntropyMap::BuildLogLUT] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

}    // namespace gc
