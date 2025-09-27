#include "octorefine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>
#include <numeric>

OctoRefine::OctoRefine() {}

cv::Point2f OctoRefine::getPointProjection(const cv::Point2f& P, const cv::Point2f& A, const cv::Point2f& B) {
    cv::Point2f AP = P - A;
    cv::Point2f AB = B - A;
    float t = (AP.dot(AB)) / (AB.dot(AB));
    t = std::clamp(t, 0.0f, 1.0f);
    return A + t * AB;
}

std::vector<cv::Point> OctoRefine::getLinePixels(const cv::Point2f& p1, const cv::Point2f& p2) {
    std::vector<cv::Point> points;
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    int steps = static_cast<int>(std::max(std::abs(dx), std::abs(dy)));
    if (steps == 0) {
        points.push_back(cv::Point(cvRound(p1.x), cvRound(p1.y)));
        return points;
    }
    float x_inc = dx / steps;
    float y_inc = dy / steps;
    float x = p1.x, y = p1.y;
    for (int i = 0; i <= steps; ++i) {
        points.emplace_back(cvRound(x), cvRound(y));
        x += x_inc;
        y += y_inc;
    }
    std::sort(points.begin(), points.end(), [](const cv::Point& a, const cv::Point& b) {
        return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
    });
    return points;
}

std::vector<std::vector<std::pair<cv::Point, cv::Point>>> OctoRefine::calculateFacetLinesN(
    const cv::Point2f& center_point,
    const std::vector<cv::Point2f>& octagon_points,
    int n
) {
    std::vector<std::vector<std::pair<cv::Point, cv::Point>>> facet_line_sets;
    int num_points = static_cast<int>(octagon_points.size());
    for (int i = 0; i < num_points; ++i) {
        cv::Point2f P_i = octagon_points[i];
        cv::Point2f P_ip1 = octagon_points[(i + 1) % num_points];
        cv::Point2f v_edge = P_ip1 - P_i;
        cv::Point2f normal_vector(-v_edge.y, v_edge.x);
        cv::Point2f v_i_to_C = center_point - P_i;
        if (normal_vector.dot(v_i_to_C) < 0)
            normal_vector = -normal_vector;
        cv::Point2f U_normal = normal_vector * (1.0f / cv::norm(normal_vector));
        std::vector<cv::Point> facet_pixels = getLinePixels(P_i, P_ip1);
        std::vector<std::pair<cv::Point, cv::Point>> current_facet_lines;
        for (const auto& P_facet_pixel : facet_pixels) {
            cv::Point2f P_facet_np(P_facet_pixel.x, P_facet_pixel.y);
            cv::Point2f P_inner_np = P_facet_np - U_normal * static_cast<float>(n);
            cv::Point2f P_outer_np = P_facet_np + U_normal * static_cast<float>(n);
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
        facet_line_sets.push_back(current_facet_lines);
    }
    return facet_line_sets;
}

std::vector<cv::Point> OctoRefine::getLineCoords(const cv::Point& pt1, const cv::Point& pt2) {
    std::vector<cv::Point> coords;
    int x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
    int dx = x2 - x1, dy = y2 - y1;
    int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps == 0) {
        coords.push_back(pt1);
        return coords;
    }
    float x_inc = static_cast<float>(dx) / steps;
    float y_inc = static_cast<float>(dy) / steps;
    float x = x1, y = y1;
    for (int i = 0; i <= steps; ++i) {
        coords.emplace_back(cvRound(x), cvRound(y));
        x += x_inc;
        y += y_inc;
    }
    std::sort(coords.begin(), coords.end(), [](const cv::Point& a, const cv::Point& b) {
        return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
    });
    coords.erase(std::unique(coords.begin(), coords.end()), coords.end());
    return coords;
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

cv::Point2f OctoRefine::findSubpixelFallingEdge(
    const cv::Mat& image,
    const cv::Point& pt1,
    const cv::Point& pt2,
    double sigma
) {
    std::vector<cv::Point> coords = getLineCoords(pt1, pt2);
    if (coords.size() < 3)
        return cv::Point2f(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
    std::vector<float> intensities;
    for (const auto& p : coords) {
        if (p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows)
            intensities.push_back(static_cast<float>(image.at<uchar>(p)));
        else
            intensities.push_back(0.0f);
    }
    std::vector<float> smoothed = gaussianSmooth1D(intensities, sigma);
    std::vector<float> gradient(smoothed.size());
    for (size_t i = 1; i < smoothed.size() - 1; ++i)
        gradient[i] = (smoothed[i + 1] - smoothed[i - 1]) / 2.0f;
    gradient[0] = smoothed[1] - smoothed[0];
    gradient[gradient.size() - 1] = smoothed[gradient.size() - 1] - smoothed[gradient.size() - 2];
    auto min_it = std::min_element(gradient.begin(), gradient.end());
    int k = static_cast<int>(std::distance(gradient.begin(), min_it));
    if (k <= 0 || k >= static_cast<int>(gradient.size()) - 1)
        return cv::Point2f(static_cast<float>(coords[k].x), static_cast<float>(coords[k].y));
    float y0 = gradient[k - 1], y1 = gradient[k], y2 = gradient[k + 1];
    float denom = 2.0f * (y0 - 2.0f * y1 + y2);
    float subpixel_idx = static_cast<float>(k);
    if (std::abs(denom) > 1e-6f)
        subpixel_idx = k + (y0 - y2) / denom;
    float frac = subpixel_idx / (coords.size() - 1);
    cv::Point2f pt1f(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
    cv::Point2f pt2f(static_cast<float>(pt2.x), static_cast<float>(pt2.y));
    cv::Point2f subpixel_pos = pt1f + (pt2f - pt1f) * frac;
    return subpixel_pos;
}

std::vector<std::vector<std::pair<cv::Point, cv::Point>>> OctoRefine::refineFindEx(
    const std::vector<cv::Point2f>& pts,
    int extension
) {
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : pts) {
        center.x += p.x;
        center.y += p.y;
    }
    center.x /= pts.size();
    center.y /= pts.size();
    return calculateFacetLinesN(center, pts, extension);
}

cv::Point2f OctoRefine::findLineIntersection(const LineEquation& lineA, const LineEquation& lineB, bool& valid) {
    float vxA = lineA.direction[0], vyA = lineA.direction[1];
    float x0A = lineA.point.x, y0A = lineA.point.y;
    float vxB = lineB.direction[0], vyB = lineB.direction[1];
    float x0B = lineB.point.x, y0B = lineB.point.y;
    cv::Point2f p0A(x0A, y0A), p0B(x0B, y0B);
    cv::Point2f b = p0B - p0A;
    cv::Matx22f M(vxA, -vxB, vyA, -vyB);
    float det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
    if (std::abs(det) < 1e-6f) {
        valid = false;
        return cv::Point2f();
    }
    cv::Vec2f rhs(b.x, b.y);
    cv::Vec2f sol = M.inv() * rhs;
    float tA = sol[0];
    cv::Point2f vA(vxA, vyA);
    valid = true;
    return p0A + tA * vA;
}

std::vector<cv::Point2f> OctoRefine::sortOctagonPoints(const std::vector<cv::Point2f>& points) {
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& p : points) {
        center.x += p.x;
        center.y += p.y;
    }
    center.x /= points.size();
    center.y /= points.size();
    auto min_y = std::min_element(points.begin(), points.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return (a.y < b.y) || (a.y == b.y && a.x < b.x);
        });
    std::vector<std::pair<float, cv::Point2f>> angles;
    for (const auto& p : points) {
        float angle = std::atan2(p.y - center.y, p.x - center.x);
        angles.emplace_back(angle, p);
    }
    std::sort(angles.begin(), angles.end(),
        [](const std::pair<float, cv::Point2f>& a, const std::pair<float, cv::Point2f>& b) {
            return a.first < b.first;
        });
    int start_idx = 0;
    for (size_t i = 0; i < angles.size(); ++i) {
        if (angles[i].second == *min_y) {
            start_idx = static_cast<int>(i);
            break;
        }
    }
    std::vector<cv::Point2f> sorted_points;
    for (size_t i = 0; i < angles.size(); ++i)
        sorted_points.push_back(angles[(start_idx + i) % angles.size()].second);
    std::reverse(sorted_points.begin(), sorted_points.end());
    return sorted_points;
}

std::vector<cv::Point2f> OctoRefine::getOctagonVertices(const std::vector<LineEquation>& lineEquations) {
    if (lineEquations.size() != 8)
        throw std::runtime_error("Input must contain exactly 8 line equations.");
    std::vector<cv::Point2f> vertices;
    for (int i = 0; i < 8; ++i) {
        bool valid = false;
        cv::Point2f pt = findLineIntersection(lineEquations[i], lineEquations[(i + 1) % 8], valid);
        if (valid)
            vertices.push_back(pt);
        else
            throw std::runtime_error("Parallel lines found in octagon vertex computation.");
    }
    return sortOctagonPoints(vertices);
}
