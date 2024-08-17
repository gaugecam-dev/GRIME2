#ifndef SEARCHLINES_H
#define SEARCHLINES_H

#include "gc_types.h"
#include <opencv2/core.hpp>

static const double MIN_SEARCH_LINE_LENGTH = 120.0;

namespace gc
{

class SearchLines
{
public:
    SearchLines();
    GC_STATUS CalcSearchLines( std::vector< cv::Point > &searchLineCorners, std::vector< LineEnds > &searchLines );
    GC_STATUS GetLineEquation( const cv::Point2d pt1, const cv::Point2d pt2, double &slope, double &intercept );
};

} // namespace gc

#endif // SEARCHLINES_H
