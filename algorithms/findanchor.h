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

#ifndef FINDANCHOR_H
#define FINDANCHOR_H

#include "gc_types.h"

namespace gc
{

class RotatedModel
{
public:
    RotatedModel() :
        angle( -9999999.0 ),
        offset( cv::Point( -1, -1 ) ),
        score( -9999999.0 )
    {}
    RotatedModel( const cv::Mat &rotModel, const double angleDeg ) :
        angle( angleDeg ),
        offset( cv::Point( -1, -1 ) ),
        score( -9999999.0 )
    {
        model = rotModel.clone();
    }

    cv::Mat model;
    double angle;
    cv::Point offset;
    double score;
};

class FindAnchor
{
public:
    FindAnchor();

    GC_STATUS SetRef( const std::string imgFilepath, const cv::Rect &modelROI );
    GC_STATUS Find( const cv::Mat &img, double &angle, cv::Point &offset );
    GC_STATUS CalcMoveModel(const cv::Mat &img, cv::Point &ptOrig, cv::Point &ptMove, double &angle );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const cv::Point2d ptCenter, const double angle );
    GC_STATUS CalcMove( const cv::Mat &img, cv::Point &ptOrig, cv::Point &ptMove );

    cv::Rect &ModelRect() { return modelRect; }
    std::string ModelRefImagePath() { return modelRefImageFilepath; }

private:
    cv::Rect rectVert;
    cv::Rect rectHoriz;
    int morphCountVert;
    int morphCountHoriz;
    bool isDarkSparseVert;
    bool isDarkSparseHoriz;

    double angleRef;
    cv::Point offsetRef;
    cv::Rect modelRect;
    std::vector< RotatedModel > rotModelSet;
    std::string modelRefImageFilepath;

    cv::Mat matProbSpace;

    void clear();

    bool isInitializedVertHoriz();
    bool isInitializedModel();
    GC_STATUS FindModel( const cv::Mat &img, double &angle, cv::Point &offset );
    GC_STATUS FindHoriz( const cv::Mat &img, cv::Point &ptA, cv::Point &ptB );
    GC_STATUS FindVert( const cv::Mat &img, cv::Point &ptA, cv::Point &ptB );
    GC_STATUS SetRef( const cv::Mat &img, const cv::Rect &modelROI );
    GC_STATUS SetRef( const cv::Mat &img, const std::vector< cv::Point > regionV, const std::vector< cv::Point > regionH,
                      const bool darkSparseV, const bool darkSparseH, const int morphCountV, const int morphCountH );
};

} //namespace gc

#endif // FINDANCHOR_H
