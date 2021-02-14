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
#include "areafeatures.h"
#include <opencv2/imgproc.hpp>
#include "entropymap.h"

using namespace cv;
using namespace std;

// Moment equation: https://en.wikipedia.org/wiki/Image_moment

namespace gc
{

AreaFeatures::AreaFeatures()
{

}
GC_STATUS AreaFeatures::CalcImageFeatures( const Mat &img, ImageAreaFeatures &feats, const Mat &mask )
{
    GC_STATUS retVal = CalcGray( img, feats.grayStats, mask );
    if ( GC_OK == retVal )
    {
        retVal = CalcEntropy( img, feats.entropyStats, mask );
        if ( GC_OK == retVal )
        {
            if ( CV_8UC3 == img.type() )
            {
                retVal = CalcHSV( img, feats.hsvStats, mask );
            }
            else
            {
                feats.hsvStats.clear();
            }
        }
    }

    return retVal;
}
GC_STATUS AreaFeatures::CalcMaskedFeatures( const cv::Mat &img, const std::vector< LabelROIItem > &rois, std::vector< ImageAreaFeatures > &areaFeatures )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        areaFeatures.clear();

        Rect roi;
        ImageAreaFeatures feats;

        Mat mask = Mat::zeros( img.size(), CV_8UC1 );
        for ( size_t i = 0; i < rois.size(); ++i )
        {
            if ( rois[ i ].contour.empty() )
            {
                FILE_LOG( logWARNING ) << "No contour available for ROI " << rois[ i ].name;
            }
            else
            {
                roi = boundingRect( rois[ i ].contour );
                mask( roi ) = 0;

                drawContours( mask, vector< vector< Point > >( 1, rois[ i ].contour ), static_cast< int >( i ), Scalar( 255 ), FILLED );

                if ( GC_OK == retVal )
                {
                    retVal = CalcImageFeatures( img( roi ), feats, mask( roi ) );
                    if ( GC_OK != retVal )
                    {
                        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcMaskedFeatures] Failed on mask " << i;
                        retVal = GC_ERR;
                        break;
                    }
                    else
                    {
                        areaFeatures.push_back( feats );
                    }
                }
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcMaskedFeatures] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS AreaFeatures::CalcGray( const Mat &img, PixelStats &stats, const Mat &mask )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat gray;
        if ( CV_8UC3 == img.type() )
        {
            cvtColor( img, gray, COLOR_BGR2GRAY );
        }
        else if ( CV_8UC1 == img.type() )
        {
            gray = img;
        }
        else
        {
            FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] Invalid image type: "
                                 << img.type() << " must be 8-bit gray or BGR";
            retVal = GC_ERR;
        }
        Scalar meanGray, stdDevGray;
        meanStdDev( gray, meanGray, stdDevGray, mask );
        stats.average = meanGray.val[ 0 ];
        stats.sigma = stdDevGray.val[ 0 ];
        retVal = CalcCentroid( gray, stats.centroid, mask );
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS AreaFeatures::CalcEntropy( const Mat &img, PixelStats &stats, const Mat &mask )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat gray;
        if ( CV_8UC3 == img.type() )
        {
            cvtColor( img, gray, COLOR_BGR2GRAY );
        }
        else if ( CV_8UC1 == img.type() )
        {
            gray = img;
        }
        else
        {
            FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] Invalid image type: " << img.type();
            retVal = GC_ERR;
        }

        Mat dst;
        EntropyMap entropyMap;
        Scalar meanGray, stdDevGray;

        retVal = entropyMap.CalcMap( gray, dst, 5, true );
        meanStdDev( dst, meanGray, stdDevGray, mask );
        stats.average = meanGray.val[ 0 ];
        stats.sigma = stdDevGray.val[ 0 ];
        retVal = CalcCentroid( gray, stats.centroid, mask );
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcEntropy] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS AreaFeatures::CalcHSV( const Mat &img, vector< PixelStats > &hsvStats, const Mat &mask )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( CV_8UC3 != img.type() )
        {
            FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] Invalid image type: "
                                 << img.type() << " must be 8-bit BGR";
            retVal = GC_ERR;
        }
        else
        {
            Mat hsvImg;
            cvtColor( img, hsvImg, COLOR_BGR2Lab );
            vector< Mat > splitHSV;
            split( hsvImg, splitHSV );

            hsvStats.clear();
            Scalar meanVal, stdDevVal;

            // lightness
            Point2d centroid;
            meanStdDev( splitHSV[ 0 ], meanVal, stdDevVal, mask );
            retVal = CalcCentroid( splitHSV[ 0 ], centroid, mask );
            if ( GC_OK == retVal )
            {
                hsvStats.push_back( PixelStats( meanVal[ 0 ], stdDevVal.val[ 0 ], centroid, -9999999.0, -9999999.0 ) );

                meanStdDev( splitHSV[ 1 ], meanVal, stdDevVal, mask );
                retVal = CalcCentroid( splitHSV[ 1 ], centroid, mask );
                if ( GC_OK == retVal )
                {
                    hsvStats.push_back( PixelStats( meanVal[ 0 ], stdDevVal.val[ 0 ], centroid, -9999999.0, -9999999.0 ) );

                    meanStdDev( splitHSV[ 2 ], meanVal, stdDevVal, mask );
                    retVal = CalcCentroid( splitHSV[ 2 ], centroid, mask );
                    if ( GC_OK == retVal )
                    {
                        hsvStats.push_back( PixelStats( meanVal[ 0 ], stdDevVal.val[ 0 ], centroid, -9999999.0, -9999999.0 ) );
                    }
                }
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcCIELab] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS AreaFeatures::CalcSobelFeatures( const cv::Mat &img, EdgeStats edgeStats, cv::Mat &mask )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( CV_8UC1 != img.type() )
        {
            FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcSobelFeatures] Invalid image type: "
                                 << img.type() << " must be 8-bit gray";
            retVal = GC_ERR;
        }
        else if ( !mask.empty() )
        {
            if ( ( img.size() != mask.size() ) || ( CV_8UC1 != mask.type() ) )
            {
                FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcSobelFeatures] Invalid mask type or size: "
                                     << img.type() << " must be 8-bit gray and same size as input image";
                retVal = GC_ERR;
            }
            else
            {
                Mat x_grad, y_grad;
                Sobel( img, x_grad, CV_16SC1, 1, 0 );
                Sobel( img, y_grad, CV_16SC1, 0, 1 );

                Mat mag( img.size(), CV_8UC1 );
                Mat dir( img.size(), CV_8UC1 );

                uchar *pPixMag = mag.data;
                uchar *pPixDir = mag.data;
                int16_t *pPixMagX = reinterpret_cast< int16_t * >( x_grad.data );
                int16_t *pPixMagY = reinterpret_cast< int16_t * >( y_grad.data );
                for ( int row = 0; row < x_grad.rows; ++row )
                {
                    for ( int col = 0; col < x_grad.cols; ++col )
                    {
                        pPixMag[ col ] = cvRound( sqrt( pPixMagX[ col ] * pPixMagX[ col ] +
                                                        pPixMagY[ col ] * pPixMagY[ col ] ) );
                        pPixDir[ col ] = cvRound( 255.0 * ( atan( pPixMagX[ col ] / pPixMagY[ col ] ) + CV_PI / 2.0 ) / CV_PI );
                    }
                    pPixMag += mag.step;
                    pPixDir += dir.step;
                    pPixMagX += x_grad.step;
                    pPixMagY += y_grad.step;
                }

                Moments mu = moments( mask.empty() ? img : img & mask, false );
                edgeStats.centroid = Point2d( mu.m10 / mu.m00, mu.m01 / mu.m00 );
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcCIELab] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS AreaFeatures::CalcCentroid( const cv::Mat &img, cv::Point2d &centroid, const cv::Mat &mask )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( CV_8UC1 != img.type() )
        {
            FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] Invalid image type: "
                                 << img.type() << " must be 8-bit gray";
            retVal = GC_ERR;
        }
        else if ( !mask.empty() )
        {
            if ( ( img.size() != mask.size() ) || ( CV_8UC1 != mask.type() ) )
            {
                FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcGray] Invalid mask type or size: "
                                     << img.type() << " must be 8-bit gray and same size as input image";
                retVal = GC_ERR;
            }
            else
            {
                Moments mu = moments( mask.empty() ? img : img & mask, false );
                centroid = Point2d( mu.m10 / mu.m00, mu.m01 / mu.m00 );
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[AreaImageFeatures::CalcCIELab] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace water
