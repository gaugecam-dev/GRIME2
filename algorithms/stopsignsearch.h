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
    StopSignTemplateSet() : pointAngle( -1 ) {}
    StopSignTemplateSet( const int ptAngle ) : pointAngle( ptAngle ) {}

    int pointAngle;
    std::vector< StopSignTemplate > ptTemplates;
};

class StopsignSearch
{
public:
    StopsignSearch();

    GC_STATUS Init( const int templateDim, const int rotateCnt );
    GC_STATUS Find( const cv::Mat &img, std::vector< cv::Point2d > &pts );

private:
    std::vector< StopSignTemplateSet > templates;

    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS DrawCorner( const int templateDim, cv::Mat &templ, cv::Mat &mask, cv::Point2d &center );
    GC_STATUS RotatePointTemplates( const size_t idx, const double angle );
    GC_STATUS CreatePointTemplates( const int templateDim, const int rotateCnt, std::vector< StopSignTemplate > &ptTemplates );
    GC_STATUS CreateTemplateOverlay( const std::string debugFolder );
};

} // namespace gc

#endif // STOPSIGNSEARCH_H
