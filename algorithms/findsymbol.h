#ifndef FINDSYMBOL_H
#define FINDSYMBOL_H

#include "gc_types.h"
#include <opencv2/core.hpp>

namespace gc
{

static const int SYMBOL_TEMPL_WIDTH = 64;
static const double SYMBOL_TEMPL_ANGLE_MAX = 7.5;
static const double SYMBOL_TEMPL_ANGLE_INC = 0.5;
static const int SYMBOL_SEARCH_IMAGE_WIDTH_START = 640;
static const int SYMBOL_SEARCH_IMAGE_WIDTH_END = 1024;
static const int SYMBOL_SEARCH_IMAGE_WIDTH_INC = 32;

class SymbolMatch
{
public:
    SymbolMatch() :
        pt( cv::Point( -1, -1 ) ),
        score( -1.0 ),
        width( -1 ),
        angleIdx( -1 )
    {}

    SymbolMatch( cv::Point point, double matchScore, int matchWidth, int matchAngleIdx ) :
        pt( point ),
        score( matchScore ),
        width( matchWidth ),
        angleIdx( matchAngleIdx )
    {}

    void clear()
    {
        pt = cv::Point( -1, -1 );
        score = -1.0;
        width = -1;
        angleIdx = -1;
    }

    cv::Point pt;
    double score;
    int width;
    int angleIdx;
};

class SymbolCandidate
{
public:
    SymbolCandidate() :
        area( -1.0 ),
        elongation( -1.0 )
    {}

    SymbolCandidate( std::vector< cv::Point > &vecPts, double contourArea, double contourElongation ) :
        contour( vecPts ),
        area( contourArea ),
        elongation( contourElongation )
    {}

    void clear()
    {
        contour.clear();
        area = -1.0;
        elongation = -1.0;
    }

    std::vector< cv::Point > contour;
    double area;
    double elongation;
};


class FindSymbol
{
public:
    FindSymbol();
    GC_STATUS Find( const cv::Mat &img, std::vector< cv::Point > &symbolPoints );
    void clear() { symbolTemplates.clear(); }

private:
    std::vector< cv::Mat > symbolTemplates;

    GC_STATUS FindRed( const cv::Mat &img, cv::Mat1b &redMask, std::vector< SymbolCandidate > &symbolCandidates );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS FindCenter( const cv::Mat &img, SymbolMatch &matchResult );
    GC_STATUS FindSymbolCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, std::vector< cv::Point > &corners );
    GC_STATUS CreateSymbolTemplates( const cv::Mat &refTemplate );
    GC_STATUS GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts );
};

} // namespace gc

#endif // FINDSYMBOL_H
