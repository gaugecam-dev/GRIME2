#include "log.h"
#include "octorefine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>

#ifndef OCTOREFINE_DEBUG
#define OCTOREFINE_DEBUG
char DEBUG_FOLDER[] = "/var/tmp/gaugecam/";
#endif

using namespace cv;
using namespace std;

namespace gc
{

OctoRefine::OctoRefine() {}

gc::GC_STATUS OctoRefine::RefinePoints( const cv::Mat &img, const std::vector< cv::Point2d > &pts,
                                        std::vector< cv::Point2d > &vertices, int minFacetPts, double sigma )
{
    gc::GC_STATUS retVal = gc::GC_OK;
    try
    {
#ifdef OCTOREFINE_DEBUG
        cv::Mat color;
        cvtColor( img, color, COLOR_GRAY2BGR );
#endif
        // Compute center
        cv::Point2d center( 0.0, 0.0 ) ;
        for (const auto& p : pts)
        {
#ifdef OCTOREFINE_DEBUG
            circle( color, p, 5, Scalar( 0, 0, 255 ), 1 );
#endif
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= pts.size();
        center.y /= pts.size();
#ifdef OCTOREFINE_DEBUG
        circle( color, center, 7, Scalar( 0, 255, 255 ), 2 );
        imwrite( string( DEBUG_FOLDER ) + string( "000_input_image.png" ), color );
#endif



        // Get facet lines
        std::vector <std::vector< std::pair< cv::Point, cv::Point > > > extendedLines;
        retVal = RefineFindExtend( pts, extendedLines );
        if ( GC_OK == retVal )
        {
            std::vector< LineEquation> lineEquations;
            for ( const auto& facetSet : extendedLines )
            {
                std::vector<cv::Point2d> facetPts;
                for (const auto& endpoints : facetSet)
                {
                    cv::Point p1 = endpoints.first;
                    cv::Point p2 = endpoints.second;
#ifdef OCTOREFINE_DEBUG
                    line( color, p1, p2, Scalar( 0, 255, 0 ), 1 );
                    imwrite( string( DEBUG_FOLDER ) + string( "010_lines.png" ), color );
#endif
                    // Ensure p1 is closer to center than p2
                    if (cv::norm(cv::Point2d(p1) - center) > cv::norm(cv::Point2d(p2) - center))
                    {
                        std::swap(p1, p2);
                    }
                    cv::Point2d edgePt;
                    retVal = FindSubpixelFallingEdge( img, p1, p2, edgePt, sigma );
                    if (edgePt.x != edgePt.x || edgePt.y != edgePt.y || GC_OK != retVal ) // NaN check
                    {
                        retVal = GC_OK;
                    }
                    else
                    {
                        facetPts.push_back(edgePt);
                    }
                }
                if (facetPts.size() >= static_cast<size_t>(minFacetPts))
                {
                    cv::Vec4f line;
                    cv::fitLine(facetPts, line, cv::DIST_L2, 0, 0.01, 0.01);
                    LineEquation eq;
                    eq.direction = cv::Vec2f(line[0], line[1]);
                    eq.point = cv::Point2d(line[2], line[3]);
                    lineEquations.push_back(eq);
                }
            }

            // Compute vertices from line equations
            retVal = GetOctagonVertices( lineEquations, vertices );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::RefinePoints] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::GetPointProjection(const cv::Point2d& P, const cv::Point2d& A, const cv::Point2d& B, cv::Point2d &C )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cv::Point2d AP = P - A;
        cv::Point2d AB = B - A;
        float t = (AP.dot(AB)) / (AB.dot(AB));
        t = std::clamp(t, 0.0f, 1.0f);
        C = A + t * AB;
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::GetPointProjection] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::GetLinePixels(const cv::Point2d& p1, const cv::Point2d& p2, std::vector<cv::Point> &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        pts.clear();
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        int steps = static_cast<int>(std::max(std::abs(dx), std::abs(dy)));
        if ( steps == 0 )
        {
            pts.push_back(cv::Point(cvRound(p1.x), cvRound(p1.y)));
        }
        else
        {
            float x_inc = dx / steps;
            float y_inc = dy / steps;
            float x = p1.x, y = p1.y;
            for (int i = 0; i <= steps; ++i) {
                pts.emplace_back(cvRound(x), cvRound(y));
                x += x_inc;
                y += y_inc;
            }
            std::sort(pts.begin(), pts.end(), [](const cv::Point& a, const cv::Point& b) {
                return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
            });
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::GetLinePixels] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::CalculateFacetLinesN( const cv::Point2d& center_point, const std::vector<cv::Point2d>& octagon_points,
                                            std::vector< std::vector< std::pair< cv::Point, cv::Point > > > &facetLineSets, int extension )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        facetLineSets.clear();
        int num_points = static_cast<int>(octagon_points.size());
        for (int i = 0; i < num_points; ++i)
        {
            cv::Point2d P_i = octagon_points[i];
            cv::Point2d P_ip1 = octagon_points[(i + 1) % num_points];
            cv::Point2d v_edge = P_ip1 - P_i;
            cv::Point2d normal_vector(-v_edge.y, v_edge.x);
            cv::Point2d v_i_to_C = center_point - P_i;
            if ( normal_vector.dot(v_i_to_C) < 0 )
            {
                normal_vector = -normal_vector;
            }
            cv::Point2d U_normal = normal_vector * (1.0f / cv::norm(normal_vector));
            std::vector<cv::Point> facet_pixels;
            retVal = GetLinePixels(P_i, P_ip1, facet_pixels );
            if ( GC_OK != retVal )
            {
                break;
            }
            std::vector<std::pair<cv::Point, cv::Point>> current_facet_lines;
            for (const auto& P_facet_pixel : facet_pixels) {
                cv::Point2d P_facet_np(P_facet_pixel.x, P_facet_pixel.y);
                cv::Point2d P_inner_np = P_facet_np - U_normal * static_cast<float>( extension );
                cv::Point2d P_outer_np = P_facet_np + U_normal * static_cast<float>( extension );
                current_facet_lines.emplace_back(
                    cv::Point(cvRound(P_inner_np.x), cvRound(P_inner_np.y)),
                    cv::Point(cvRound(P_outer_np.x), cvRound(P_outer_np.y))
                );
            }
            int start = static_cast<int>(current_facet_lines.size() / 8);
            if (start < static_cast<int>(current_facet_lines.size()))
                current_facet_lines = std::vector<std::pair<cv::Point, cv::Point>>(
                    current_facet_lines.begin() + start,
                    current_facet_lines.end() - start
                );
            facetLineSets.push_back(current_facet_lines);
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::CalculateFacetLinesN] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::GetLineCoords(const cv::Point& pt1, const cv::Point& pt2, std::vector<cv::Point> &coords )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        coords.clear();
        int x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
        int dx = x2 - x1, dy = y2 - y1;
        int steps = std::max(std::abs(dx), std::abs(dy));
        if (steps == 0)
        {
            coords.push_back(pt1);
        }
        else
        {
            float x_inc = static_cast<float>(dx) / steps;
            float y_inc = static_cast<float>(dy) / steps;
            float x = x1, y = y1;
            for (int i = 0; i <= steps; ++i)
            {
                coords.emplace_back(cvRound(x), cvRound(y));
                x += x_inc;
                y += y_inc;
            }
            std::sort(coords.begin(), coords.end(), [](const cv::Point& a, const cv::Point& b) {
                return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
            });
            coords.erase(std::unique(coords.begin(), coords.end()), coords.end());
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::GetLineCoords] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

std::vector<float> OctoRefine::gaussianSmooth1D(const std::vector<float>& intensities, double sigma) {
    cv::Mat src(intensities.size(), 1, CV_32F, const_cast<float*>(intensities.data()));
    cv::Mat dst;
    int ksize = std::max(3, int(6 * sigma + 1) | 1);
    cv::GaussianBlur(src, dst, cv::Size(ksize, 1), sigma, 0, cv::BORDER_REPLICATE);
    std::vector<float> result(dst.rows);
    for (int i = 0; i < dst.rows; ++i)
        result[i] = dst.at<float>(i, 0);
    return result;
}

GC_STATUS OctoRefine::FindSubpixelFallingEdge( const cv::Mat& image, const cv::Point& pt1,
                                               const cv::Point& pt2, Point2d &subpixel_pos, double sigma )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        std::vector<cv::Point> coords;
        retVal = GetLineCoords( pt1, pt2, coords );
        if ( GC_OK == retVal )
        {
            if (coords.size() < 3)
            {
                subpixel_pos = cv::Point2d(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
            }
            else
            {
                std::vector<float> intensities;
                for (const auto& p : coords)
                {
                    if (p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows)
                    {
                        intensities.push_back(static_cast<float>(image.at<uchar>(p)));
                    }
                    else
                    {
                        intensities.push_back(0.0f);
                    }
                }
                std::vector<float> smoothed = gaussianSmooth1D(intensities, sigma);
                std::vector<float> gradient(smoothed.size());
                for (size_t i = 1; i < smoothed.size() - 1; ++i)
                {
                    gradient[i] = (smoothed[i + 1] - smoothed[i - 1]) / 2.0;
                }
                gradient[0] = smoothed[1] - smoothed[0];
                gradient[gradient.size() - 1] = smoothed[gradient.size() - 1] - smoothed[gradient.size() - 2];
                auto min_it = std::min_element(gradient.begin(), gradient.end());
                int k = static_cast<int>(std::distance(gradient.begin(), min_it));
                if (k <= 0 || k >= static_cast<int>(gradient.size()) - 1)
                {
                    subpixel_pos = cv::Point2d(static_cast<float>(coords[k].x), static_cast<float>(coords[k].y));
                }
                else
                {
                    float y0 = gradient[k - 1], y1 = gradient[k], y2 = gradient[k + 1];
                    float denom = 2.0f * (y0 - 2.0f * y1 + y2);
                    float subpixel_idx = static_cast<float>(k);
                    if (std::abs(denom) > 1e-6f)
                    {
                        subpixel_idx = k + (y0 - y2) / denom;
                    }
                    float frac = subpixel_idx / (coords.size() - 1);
                    cv::Point2d pt1f(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
                    cv::Point2d pt2f(static_cast<float>(pt2.x), static_cast<float>(pt2.y));
                    subpixel_pos = pt1f + (pt2f - pt1f) * frac;
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::findSubpixelFallingEdge] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

gc::GC_STATUS OctoRefine::RefineFindExtend( const std::vector<cv::Point2d>& pts,
                                            std::vector <std::vector< std::pair< cv::Point, cv::Point > > > &extendedLines,
                                            const int extension )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cv::Point2d center(0.0f, 0.0f);
        for (const auto& p : pts)
        {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= pts.size();
        center.y /= pts.size();
        retVal = CalculateFacetLinesN( center, pts, extendedLines, extension );
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::RefineFindExtend] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::FindLineIntersection( const LineEquation& lineA, const LineEquation& lineB, bool& valid, cv::Point2d &pt )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        float vxA = lineA.direction[0], vyA = lineA.direction[1];
        float x0A = lineA.point.x, y0A = lineA.point.y;
        float vxB = lineB.direction[0], vyB = lineB.direction[1];
        float x0B = lineB.point.x, y0B = lineB.point.y;
        cv::Point2d p0A(x0A, y0A), p0B(x0B, y0B);
        cv::Point2d b = p0B - p0A;
        cv::Matx22f M(vxA, -vxB, vyA, -vyB);
        float det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        if ( std::abs(det) < 1e-6f )
        {
            valid = false;
            FILE_LOG( logERROR ) << "[OctoRefine::RefineFindExtend] No viable conversion to type cv::Point2d";
            retVal = GC_ERR;
        }
        else
        {
            cv::Vec2f rhs(b.x, b.y);
            cv::Vec2f sol = M.inv() * rhs;
            float tA = sol[0];
            cv::Point2d vA(vxA, vyA);
            valid = true;
            pt = p0A + tA * vA;
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::RefineFindExtend] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::SortOctagonPoints( const std::vector< cv::Point2d > &points, std::vector< cv::Point2d > &ptsSorted )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ptsSorted.clear();
        cv::Point2d center(0.0f, 0.0f);
        for (const auto& p : points)
        {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= points.size();
        center.y /= points.size();
        auto min_y = std::min_element(points.begin(), points.end(),
            [](const cv::Point2d& a, const cv::Point2d& b) {
                return (a.y < b.y) || (a.y == b.y && a.x < b.x);
            });
        std::vector<std::pair<float, cv::Point2d>> angles;
        for (const auto& p : points)
        {
            float angle = std::atan2(p.y - center.y, p.x - center.x);
            angles.emplace_back(angle, p);
        }
        std::sort(angles.begin(), angles.end(),
            [](const std::pair<float, cv::Point2d>& a, const std::pair<float, cv::Point2d>& b) {
                return a.first < b.first;
            });
        int start_idx = 0;
        for (size_t i = 0; i < angles.size(); ++i)
        {
            if (angles[i].second == *min_y)
            {
                start_idx = static_cast<int>(i);
                break;
            }
        }

        for (size_t i = 0; i < angles.size(); ++i)
        {
            ptsSorted.push_back(angles[(start_idx + i) % angles.size()].second);
        }
        std::reverse(ptsSorted.begin(), ptsSorted.end());
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::SortOctagonPoints] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS OctoRefine::GetOctagonVertices(const std::vector<LineEquation>& lineEquations, std::vector<cv::Point2d> &vertsSorted )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( lineEquations.size() != 8 )
        {
            FILE_LOG( logERROR ) << "[OctoRefine::GetOctagonVertices] Input must contain exactly 8 line equations";
            retVal = GC_ERR;
        }
        else
        {
            std::vector<cv::Point2d> vertices;
            for (int i = 0; i < 8; ++i)
            {
                bool valid = false;
                cv::Point2d pt;
                retVal = FindLineIntersection( lineEquations[i], lineEquations[(i + 1) % 8], valid, pt );
                if ( GC_OK != retVal )
                {
                    break;
                }
                else if ( valid )
                {
                    vertices.push_back(pt);
                }
                else
                {
                    FILE_LOG( logERROR ) << "[OctoRefine::GetOctagonVertices] Parallel lines found in octagon vertex computation";
                    retVal = GC_ERR;
                    break;
                }
            }
            retVal = SortOctagonPoints( vertices, vertsSorted );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::GetOctagonVertices] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;

    return retVal;
}

} // namespace gc
