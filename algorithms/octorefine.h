#pragma once
#include "gc_types.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

struct LineEquation {
    cv::Vec2d direction; // (vx, vy)
    cv::Point2d point;   // (x0, y0)
};

namespace gc
{

class OctoRefine {
public:
    OctoRefine();

    // Main method
    GC_STATUS RefinePoints( const cv::Mat &img, const std::vector< cv::Point2d > &pts,
                            std::vector< cv::Point2d > &vertices, int minFacetPts = 8, double sigma = 2.0 );
    // Utility methods
    GC_STATUS GetPointProjection(const cv::Point2d& P, const cv::Point2d& A, const cv::Point2d& B, cv::Point2d &C );
    GC_STATUS GetLinePixels( const cv::Point2d& p1, const cv::Point2d& p2, const cv::Point2d center, std::vector< cv::Point > &pts );
    GC_STATUS CalculateFacetLinesN( const cv::Point2d &center, const std::vector< cv::Point2d > &octagon_points,
                                    std::vector< std::vector< std::pair< cv::Point, cv::Point> > > &facetLineSetsconst,
                                    cv::Size imgSize, int extension );
    GC_STATUS GetLineCoords( const cv::Point& pt1, const cv::Point& pt2, std::vector<cv::Point> &coords );
    GC_STATUS FindSubpixelFallingEdge( const cv::Mat& image, const cv::Point& pt1, const cv::Point& pt2,
                                       cv::Point2d &sub_pixel, double sigma = 2.0 );
    GC_STATUS RefineFindExtend( const std::vector<cv::Point2d>& pts,
                                std::vector <std::vector< std::pair< cv::Point, cv::Point > > > &extendedLines,
                                const cv::Size imgSize, const int extension = 12 );
    GC_STATUS FindLineIntersection( const LineEquation& lineA, const LineEquation& lineB, bool& valid, cv::Point2d &pt );
    GC_STATUS SortOctagonPoints( const std::vector< cv::Point2d > &points, std::vector< cv::Point2d > &ptsSorted );
    GC_STATUS GetOctagonVertices(const std::vector<LineEquation>& lineEquations, std::vector<cv::Point2d> &vertsSorted );

    // Example usage (not implemented here)
    // void processImage(const cv::Mat& image, const std::vector<cv::Point2d>& pts);

private:
    // Helper for Gaussian smoothing and gradient
    GC_STATUS GaussianSmooth1D( const std::vector<float>& intensities, double sigma, std::vector< float > &result );
    GC_STATUS DrawExtendedLine( cv::Mat& image, const cv::Vec4f& line_params, const cv::Scalar& color, int thickness );
    GC_STATUS FlattenLighting( const cv::Mat &img, cv::Mat &flattened );
    GC_STATUS CalcSubPixel( const cv::Vec3d &p1, const cv::Vec3d &p2, const cv::Vec3d &p3, cv::Point2d &subpixel_pos );
};

} // namespace gc
