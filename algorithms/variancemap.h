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

#ifndef INTEGRALIMAGE_H
#define INTEGRALIMAGE_H

#include "gc_types.h"
#include <opencv2/core/core.hpp>

namespace gc
{

class VarianceMap
{
public:
    VarianceMap();
    GC_STATUS CreateMap( const cv::Mat &src, cv::Mat &dst, const int kernSize, double floatscale = -1.0 );
    GC_STATUS CreateMap( const cv::Mat &src, cv::Mat &dst, const int kernSize, const cv::Mat &mask, double floatscale = -1.0 );

private:
    cv::Mat m_image;
    cv::Mat m_integral;
    cv::Mat m_sq_integral;
    cv::Mat m_matVariance_32F;

    //function to compute integral image
    GC_STATUS Compute( const cv::Mat &image );

    //function to compute mean value of a patch
    float CalcMean( cv::Rect r );

    //function to compute variance of a patch
    float CalcVariance( cv::Rect r );
};

} // namespace gc

#endif // INTEGRALIMAGE_H
