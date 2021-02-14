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
#include "variancemap.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace gc
{

VarianceMap::VarianceMap()
{
}

GC_STATUS VarianceMap::Compute( const cv::Mat &image )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        image.copyTo( m_image );
        cv::integral( m_image, m_integral, m_sq_integral );
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[Compute][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

float VarianceMap::CalcMean( cv::Rect r )
{
    int width = m_integral.cols;
    // int height=_integral.rows;
    unsigned int *ii1 = reinterpret_cast< unsigned int * >( m_integral.data );
    int a = r.x + ( r.y * ( width ) );
    int b = ( r.x + r.width ) + ( r.y * ( width ) );
    int c = r.x + ( ( r.y + r.height ) * ( width ) );
    int d = ( r.x + r.width ) + ( r.y + r.height ) * ( width );
    float mx = static_cast< float >( ii1[ a ] + ii1[ d ] - ii1[ b ] - ii1[ c ] );
    mx = mx / static_cast< float >( r.width * r.height );
    return mx;
}

float VarianceMap::CalcVariance( cv::Rect r )
{
    int width = m_integral.cols;
    // int height=_integral.rows;
    int a = r.x + ( r.y * width );
    int b = ( r.x + r.width ) + ( r.y * width );
    int c = r.x + ( ( r.y + r.height ) * width );
    int d = ( r.x + r.width ) + ( r.y + r.height ) * width;
    float mx = CalcMean( r );
    double *ii2 = reinterpret_cast< double * >( m_sq_integral.data );
    // double *ii2 = ( double * )( _sq_integral.data );
    float mx2 = static_cast< float >( ii2[ a ] + ii2[ d ] - ii2[ b ] - ii2[ c ] );
    mx2 = mx2 / ( r.width * r.height );
    mx2 = mx2 - ( mx * mx );
    return mx2;
}
GC_STATUS VarianceMap::CreateMap( const cv::Mat &src, cv::Mat &dst, const int kernSize, double floatscale )
{
    cv::Mat mask;
    GC_STATUS retVal = CreateMap( src, dst, kernSize, mask, floatscale );
    return retVal;
}
GC_STATUS VarianceMap::CreateMap( const cv::Mat &src, cv::Mat &dst, const int kernSize, const cv::Mat &mask, double floatscale )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( 3 > kernSize || src.rows < kernSize || src.cols < kernSize )
        {
            FILE_LOG( logERROR ) << "[CreateMap][" << __func__ << "] Invalid kernels size=" << kernSize << " (must be greater than 5, odd, and smaller than the image dimensions)";
            retVal = GC_ERR;
        }
        else
        {
            int kernelSize = ( 0 == 2 % kernSize ? kernSize + 1 : kernSize );
            int kernSizeHalf = kernelSize >> 1;
            retVal = Compute( src );
            if ( GC_OK == retVal )
            {
                m_matVariance_32F.create( src.size(), CV_32F );
                m_matVariance_32F.setTo( 0.0 );

                int col, row;
                float *pPixf;
                uchar *pPixMask = mask.data + kernSizeHalf * static_cast< int >( mask.step );
                if ( mask.empty() )
                {
                    for ( row = kernSizeHalf; row < m_matVariance_32F.rows - kernSizeHalf; ++row )
                    {
                        pPixf = &m_matVariance_32F.at< float >( cv::Point( 0, row ) );
                        for ( col = kernSizeHalf; col < m_matVariance_32F.cols - kernSizeHalf; ++col )
                        {
                            pPixf[ col ] = CalcVariance( cv::Rect( col - kernSizeHalf, row - kernSizeHalf, kernelSize, kernelSize ) );
                        }
                    }
                }
                else
                {
                    for ( row = kernSizeHalf; row < m_matVariance_32F.rows - kernSizeHalf; ++row )
                    {
                        pPixf = &m_matVariance_32F.at< float >( cv::Point( 0, row ) );
                        for ( col = kernSizeHalf; col < m_matVariance_32F.cols - kernSizeHalf; ++col )
                        {
                            if ( 0 != pPixMask[ col ] )
                                pPixf[ col ] = CalcVariance( cv::Rect( col - kernSizeHalf, row - kernSizeHalf, kernelSize, kernelSize ) );
                        }
                        pPixMask += static_cast< int >( mask.step );
                    }
                }

				double dMin, dMax;
                cv::minMaxIdx( m_matVariance_32F, &dMin, &dMax );

				if (floatscale > 0.0)
				{
                    m_matVariance_32F.convertTo( dst, dst.type(), 255.0 * floatscale / dMax );
				}
				else
                    m_matVariance_32F.convertTo( dst, dst.type(), 255.0 / dMax );
			
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[CreateMap][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
