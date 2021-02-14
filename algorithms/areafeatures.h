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

#ifndef AREAIMAGEFEATURES_H
#define AREAIMAGEFEATURES_H

#include <opencv2/core.hpp>
#include "gc_types.h"
#include "featuredata.h"
#include "labelroi.h"

namespace gc
{

class AreaFeatures
{
public:
    AreaFeatures();

    GC_STATUS CalcImageFeatures( const cv::Mat &img, ImageAreaFeatures &feats, const cv::Mat &mask );
    GC_STATUS CalcEntropy( const cv::Mat &img, PixelStats &stats, const cv::Mat &mask );
    GC_STATUS CalcGray( const cv::Mat &img, PixelStats &stats, const cv::Mat &mask );
    GC_STATUS CalcHSV( const cv::Mat &img, std::vector< PixelStats > &hsvStats, const cv::Mat &mask );
    GC_STATUS CalcCentroid( const cv::Mat &img, cv::Point2d &centroid, const cv::Mat &mask );
    GC_STATUS CalcSobelFeatures( const cv::Mat &img, EdgeStats edgeStats, cv::Mat &mask );

    GC_STATUS CalcMaskedFeatures( const cv::Mat &img, const std::vector< LabelROIItem > &rois, std::vector< ImageAreaFeatures > &areaFeatures );
};

} // namespace water

#endif // AREAIMAGEFEATURES_H
