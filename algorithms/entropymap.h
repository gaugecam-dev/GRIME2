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

#ifndef ENTROPY_MAP_H
#define ENTROPY_MAP_H

#include "gc_types.h"
#include <opencv2/core.hpp>

static const bool DEFAULT_ENTROPY_USEELLIPSE = true;
static const int DEFAULT_ENTROPY_KERN_SIZE = 8;
static const int DEFAULT_ENTROPY_MORPH_KERN_SIZE = 5;
static const int DEFAULT_ENTROPY_MORPH_ITERS = 1;
static const double DEFAULT_ENTROPY_THRESHGAIN = 1.0;
static const int DEFAULT_ENTROPY_CLEAN_CONTOUR_WIDTH = 11;

namespace gc
{

class EntropyMap
{
public:
    EntropyMap();
    ~EntropyMap();

    GC_STATUS CalcMap( const cv::Mat &src, cv::Mat &dst, const int kernelSize, const bool useEllipse = true );

private:
    GC_STATUS BuildLogLUT( const cv::Mat &mask, std::vector< float > &lut );
    GC_STATUS CalcTile( const cv::Mat &src, cv::Mat &dst, const int kernelSize, const cv::Mat &mask );
    GC_STATUS CalcEntropyValue( const cv::Mat &img, float &enter, const cv::Mat &mask );
};

}    // namespace gc

#endif // ENTROPY_MAP_H
