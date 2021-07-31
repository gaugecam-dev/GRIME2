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
class SymbolLine
{
public:
    SymbolLine() :
        pt1( cv::Point2d( -1.0, -1.0 ) ),
        pt2( cv::Point2d( -1.0, -1.0 ) )
    {}

    SymbolLine( const cv::Point2d point1, const cv::Point2d point2 ) :
        pt1( point1 ),
        pt2( point2 )
    {}

    void clear()
    {
        pt1 = cv::Point2d( -1.0, -1.0 );
        pt2 = cv::Point2d( -1.0, -1.0 );
    }

    cv::Point2d pt1;
    cv::Point2d pt2;
};

class SymbolOctagonLines
{
public:
    SymbolOctagonLines() {}

    void clear()
    {
        top.clear();
        topRight.clear();
        right.clear();
        botRight.clear();
        bot.clear();
        botLeft.clear();
        left.clear();
        topLeft.clear();
    }

    SymbolLine top;
    SymbolLine topRight;
    SymbolLine right;
    SymbolLine botRight;
    SymbolLine bot;
    SymbolLine botLeft;
    SymbolLine left;
    SymbolLine topLeft;
};

class FindSymbol
{
public:
    FindSymbol();
    GC_STATUS Load( const std::string jsonCalFilepath );
    GC_STATUS Save( const std::string jsonCalFilepath );
    GC_STATUS Calibrate( const cv::Mat &img, const double octoSideLength, const cv::Point searchLftTopPt,
                         const cv::Point searchRgtTopPt, const cv::Point searchLftBotPt, const cv::Point searchRgtBotPt );
    GC_STATUS PixelToWorld( const cv::Point2d ptPixel, cv::Point2d &ptWorld );
    GC_STATUS WorldToPixel( const cv::Point2d ptWorld, cv::Point2d &ptPixel );
    GC_STATUS DrawCalibration( const cv::Mat &img, cv::Mat &result );
    void clear();

private:
    cv::Mat matHomogPixToWorld;
    cv::Mat matHomogWorldToPix;
    SymbolCalibModel model;

    GC_STATUS FindRed( const cv::Mat &img, cv::Mat1b &redMask, std::vector< SymbolCandidate > &symbolCandidates );
    GC_STATUS RotateImage( const cv::Mat &src, cv::Mat &dst, const double angle );
    GC_STATUS GetNonZeroPoints( cv::Mat &img, std::vector< cv::Point > &pts );
    GC_STATUS GetLineEndPoints( cv::Mat &mask, const cv::Rect rect, cv::Point2d &pt1, cv::Point2d &pt2 );
    GC_STATUS LineIntersection( const SymbolLine line1, const SymbolLine line2, cv::Point2d &r );
    GC_STATUS FindCorners( const cv::Mat &mask, const std::vector< cv::Point > &contour, SymbolOctagonLines &octoLines );
    GC_STATUS FindDiagonals( const cv::Mat &mask, const std::vector< cv::Point > &contour, SymbolOctagonLines &octoLines );
    GC_STATUS CalcCorners( const SymbolOctagonLines octoLines, std::vector< cv::Point2d > &symbolCorners );
    GC_STATUS CalcOctoWorldPoints( const double sideLength, std::vector< cv::Point2d > &pts );
};

} // namespace gc

#endif // FINDSYMBOL_H
