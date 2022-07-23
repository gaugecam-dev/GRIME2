#ifndef STOPSIGNSEARCH_H
#define STOPSIGNSEARCH_H

#include "gc_types.h"
#include "opencv2/imgproc.hpp"

namespace gc
{

class StopSignTemplate
{
public:
    StopSignTemplate() :
        angle( -9999999 ),
        offset( cv::Point2d( -1.0, -1.0 ) )
    {}

    double angle;
    cv::Point2d offset;
    cv::Mat mask;
    cv::Mat templ;
};

class StopSignTemplateSet
{
public:
    StopSignTemplateSet() : pos( -1 ) {}

    int pos; // Left point of horizontal top line = 0. Clockwise.
    std::vector< StopSignTemplate > templateSet;
};

class StopsignSearch
{
public:
    StopsignSearch();

    void clear();
    GC_STATUS Find( const cv::Mat &img, std::vector< cv::Point2d > &pts );
    GC_STATUS CreatePointTemplates( const int templateDim, const int rotateCnt, std::vector< StopSignTemplate > &ptTemplates );

private:
    std::vector< StopSignTemplateSet > templates;

    GC_STATUS DrawCorner( const int templateDim, cv::Mat &templ, cv::Mat &mask, cv::Point2d &center );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
};

} // namespace gc

#endif // STOPSIGNSEARCH_H
