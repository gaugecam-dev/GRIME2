#ifndef FINDSYMBOL_H
#define FINDSYMBOL_H

#include "gc_types.h"
#include <opencv2/core.hpp>

namespace gc
{

static const int SYMBOL_TEMPL_WIDTH = 512;
static const double SYMBOL_TEMPL_ANGLE_MAX = 5.5;
static const double SYMBOL_TEMPL_ANGLE_INC = 0.5;

class FindSymbol
{
public:
    FindSymbol();
    GC_STATUS CreateSymbolTemplates( const cv::Mat &refTemplate );

private:
    std::vector< cv::Mat > symbolTemplates;

    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS FindCenter( const cv::Mat &img, cv::Point2d &center );
};

} // namespace gc

#endif // FINDSYMBOL_H
