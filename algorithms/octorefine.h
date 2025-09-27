#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

struct LineEquation {
    cv::Vec2f direction; // (vx, vy)
    cv::Point2f point;   // (x0, y0)
};

class OctoRefine {
public:
    OctoRefine();

    // Utility methods
    static cv::Point2f getPointProjection(const cv::Point2f& P, const cv::Point2f& A, const cv::Point2f& B);
    static std::vector<cv::Point> getLinePixels(const cv::Point2f& p1, const cv::Point2f& p2);
    static std::vector<std::vector<std::pair<cv::Point, cv::Point>>> calculateFacetLinesN(
        const cv::Point2f& center_point,
        const std::vector<cv::Point2f>& octagon_points,
        int n
    );
    static std::vector<cv::Point> getLineCoords(const cv::Point& pt1, const cv::Point& pt2);
    static cv::Point2f findSubpixelFallingEdge(
        const cv::Mat& image,
        const cv::Point& pt1,
        const cv::Point& pt2,
        double sigma = 2.0
    );
    static std::vector<std::vector<std::pair<cv::Point, cv::Point>>> refineFindEx(
        const std::vector<cv::Point2f>& pts,
        int extension = 20
    );
    static cv::Point2f findLineIntersection(const LineEquation& lineA, const LineEquation& lineB, bool& valid);
    static std::vector<cv::Point2f> sortOctagonPoints(const std::vector<cv::Point2f>& points);
    static std::vector<cv::Point2f> getOctagonVertices(const std::vector<LineEquation>& lineEquations);

    // Example usage (not implemented here)
    // void processImage(const cv::Mat& image, const std::vector<cv::Point2f>& pts);

private:
    // Helper for Gaussian smoothing and gradient
    static std::vector<float> gaussianSmooth1D(const std::vector<float>& intensities, double sigma);
};
