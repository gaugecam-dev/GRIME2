#include "log.h"
#include "octorefine.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>
#include <cmath>

#ifdef OCTOREFINE_DEBUG
#undef OCTOREFINE_DEBUG
char DEBUG_FOLDER[] = "/var/tmp/gaugecam/";
#endif

using namespace cv;
using namespace std;

double DISTANCE( const Point a, const Point b ) { return sqrt(static_cast<double>((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y))); }
double DISTANCE( const Point2d a, const Point2d b ) { return sqrt(static_cast<double>((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y))); }

namespace gc
{

OctoRefine::OctoRefine() {}

gc::GC_STATUS OctoRefine::RefinePoints( const cv::Mat &img, const std::vector< cv::Point2d > &pts,
                                        std::vector< cv::Point2d > &vertices, int minFacetPts, double sigma )
{
    gc::GC_STATUS retVal = gc::GC_OK;
    try
    {
        std::vector< cv::Point2d > ptsSorted;
        retVal = SortOctagonPoints( pts, ptsSorted );
        if ( GC_OK == retVal )
        {
            Mat blur;
            // GaussianBlur( img, blur, Size( 9, 9 ), 1.0 );
            medianBlur( img, blur, 7 );
#ifdef OCTOREFINE_DEBUG
            cv::Mat color;
            cvtColor( blur, color, COLOR_GRAY2BGR );
#endif
            // Compute center
            cv::Point2d center( 0.0, 0.0 ) ;
            for (const auto& p : ptsSorted)
            {
#ifdef OCTOREFINE_DEBUG
                circle( color, p, 5, Scalar( 0, 0, 255 ), 1 );
#endif
                center.x += p.x;
                center.y += p.y;
            }
            center.x /= ptsSorted.size();
            center.y /= ptsSorted.size();
            Point center_int( std::round( center.x ), std::round( center.y ) );
#ifdef OCTOREFINE_DEBUG
            circle( color, center, 7, Scalar( 0, 255, 255 ), 2 );
            imwrite( string( DEBUG_FOLDER ) + string( "000_input_image.png" ), color );
            imwrite( string( DEBUG_FOLDER ) + string( "005_input_blur.png" ), blur );
#endif

            // Get facet lines
            std::vector< cv::Point2d > facetPts;
            std::vector <std::vector< std::pair< cv::Point, cv::Point > > > extendedLines;
            retVal = RefineFindExtend( ptsSorted, extendedLines );
            if ( GC_OK == retVal )
            {
#ifdef OCTOREFINE_DEBUG
                int item = 0;
#endif
                std::vector< LineEquation> lineEquations;
                for ( const auto& facetSet : extendedLines )
                {
#ifdef OCTOREFINE_DEBUG
                    cv::Point pt1 = facetSet[ facetSet.size() >> 1 ].second;
                    cv::Point pt2 = facetSet[ facetSet.size() >> 1 ].first;
                    cout << item << " inner=" << pt1 << " outer=" << pt2 << endl;
                    putText( color, to_string( item ), pt1, FONT_HERSHEY_PLAIN, 1.0, Scalar( 255, 0, 0 ), 2 );
#endif

                    facetPts.clear();
                    for (const auto& endpoints : facetSet)
                    {
                        cv::Point p1 = endpoints.first;
                        cv::Point p2 = endpoints.second;
#ifdef OCTOREFINE_DEBUG
                        color.at< cv::Vec3b >( p1 ) = cv::Vec3b( 0, 255, 0 );
                        color.at< cv::Vec3b >( p2 ) = cv::Vec3b( 0, 0, 255 );
#endif
                        cv::Point2d edgePt;
                        retVal = FindSubpixelFallingEdge( blur, p1, p2, edgePt, sigma );
                        if ( edgePt.x != edgePt.x || edgePt.y != edgePt.y || GC_OK != retVal ) // NaN check
                        {
                            retVal = GC_OK;
                        }
                        else
                        {
                            //  std::swap( edgePt.x, edgePt.y );
                            facetPts.push_back( edgePt );
                        }
                    }
                    if ( facetPts.size() >= static_cast< size_t >( minFacetPts ) )
                    {
#ifdef OCTOREFINE_DEBUG
                        for ( size_t i = 0; i < facetPts.size(); ++i )
                        {
                            cout << "x=" << facetPts[ i ].x << " y=" << facetPts[ i ].y << endl;
                            color.at< cv::Vec3b >( facetPts[ i ].x, facetPts[ i ].y ) = Vec3b( 255, 0, 0 );
                        }
#endif
                        cv::Vec4f line;
                        cv::fitLine( facetPts, line, DIST_L1, 0, 0.01, 0.01 );
#ifdef OCTOREFINE_DEBUG
                        cout << "fitLine result=" << line << endl;
                        DrawExtendedLine( color, line, Scalar( 0, 255, 255 ), 1 );
                        imwrite( string( DEBUG_FOLDER ) + string( "090_single_line.png" ), color );
#endif

                        LineEquation eq;
                        eq.direction = cv::Vec2d( line[ 0 ], line[ 1 ]);
                        eq.point = cv::Point2d( line[ 2 ], line[ 3 ] );
                        lineEquations.push_back( eq );
                    }

#ifdef OCTOREFINE_DEBUG
                    item++;
#endif
                }

                // Compute vertices from line equations
                retVal = GetOctagonVertices( lineEquations, vertices );
#ifdef OCTOREFINE_DEBUG
                cout << "~~~~~~~~" << endl << "VERTICES" << "~~~~~~~~" << endl;
                int i = 0;
                for ( const auto& vertex : vertices )
                {
                    cout << "i=" << i++ << " pt=" << vertex << endl;
                    circle( color, vertex, 2, Scalar( 0, 0, 255 ), FILLED );
                }
                imwrite( string( DEBUG_FOLDER ) + string( "030_refined_vertices.png" ), color );
#endif
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::RefinePoints] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctoRefine::DrawExtendedLine( cv::Mat& image, const cv::Vec4f& line_params,
                                        const cv::Scalar& color, int thickness )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        float vx = line_params[ 0 ];
        float vy = line_params[ 1 ];
        float x0 = line_params[ 2 ];
        float y0 = line_params[ 3 ];

        double scale = 1000.0;

        if ( image.cols > image.rows )
        {
            scale = static_cast< double >( image.cols );
        }
        else
        {
            scale = static_cast< double >( image.rows );
        }

        double t_large = scale * 2.0;

        cv::Point P1( cvRound(x0 - t_large * vx), cvRound(y0 - t_large * vy) );
        cv::Point P2( cvRound(x0 + t_large * vx), cvRound(y0 + t_large * vy) );

        cv::line( image, P1, P2, color, thickness, cv::LINE_AA );
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::DrawExtendedLine] " << e.what();
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

GC_STATUS OctoRefine::GetLinePixels( const cv::Point2d& p1, const cv::Point2d& p2, const cv::Point2d center, std::vector< cv::Point > &pts )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        pts.clear();
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        int steps = static_cast< int >( std::max( std::abs( dx ), std::abs( dy ) ) );
        if ( steps == 0 )
        {
            pts.push_back( cv::Point( cvRound( p1.x ), cvRound( p1.y ) ) );
        }
        else
        {
            float x_inc = dx / steps;
            float y_inc = dy / steps;
            float x = p1.x, y = p1.y;
            for ( int i = 0; i <= steps; ++i )
            {
                pts.emplace_back( cvRound( x ), cvRound( y ) );
                x += x_inc;
                y += y_inc;
            }

            // std::sort( pts.begin(), pts.end(), []( const cv::Point& a, const cv::Point& b ) {
            //     return ( a.x == b.x ) ? ( a.y < b.y ) : ( a.x < b.x );
            // });

            std::sort( pts.begin(), pts.end(), [ center ]( const cv::Point2d& a, const cv::Point2d& b ) {
                return DISTANCE( a, center ) < DISTANCE( b, center );
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
bool comparePoints(const cv::Point& a, const cv::Point& b)
{
    if ( a.x != b.x )
    {
        return a.x < b.x;
    }
    return a.y < b.y;
}
GC_STATUS OctoRefine::CalculateFacetLinesN( const cv::Point2d& center, const std::vector< cv::Point2d > &octagon_points,
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
            cv::Point2d v_i_to_C = center - P_i;
            if ( normal_vector.dot(v_i_to_C) < 0 )
            {
                normal_vector = -normal_vector;
            }
            cv::Point2d U_normal = normal_vector * (1.0f / cv::norm(normal_vector));
            std::vector<cv::Point> facet_pixels;
            retVal = GetLinePixels( P_i, P_ip1, center, facet_pixels );
            if ( GC_OK != retVal )
            {
                break;
            }
            std::sort( facet_pixels.begin(), facet_pixels.end(), comparePoints );
            auto last_unique = std::unique(facet_pixels.begin(), facet_pixels.end(), [](const cv::Point& a, const cv::Point& b) {
                                               return (a.x == b.x) && (a.y == b.y); });
            facet_pixels.erase(last_unique, facet_pixels.end());
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
                current_facet_lines = std::vector< std::pair< cv::Point, cv::Point > >(
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

GC_STATUS OctoRefine::GaussianSmooth1D( const std::vector<float>& intensities, double sigma, std::vector< float > &result )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        result.clear();
        cv::Mat src( intensities.size(), 1, CV_32F, const_cast< float * >( intensities.data() ) );
        cv::Mat dst;
        int ksize = std::max( 3, int( 6 * sigma + 1 ) | 1 );
        cv::GaussianBlur( src, dst, cv::Size( ksize, 1 ), sigma, 0, cv::BORDER_REPLICATE );
        std::vector< float > result( dst.rows );
        for ( int i = 0; i < dst.rows; ++i )
        {
            result[ i ] = dst.at<float>( i, 0 );
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::GaussianSmooth1D] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}
#if 1
GC_STATUS OctoRefine::CalcSubPixel( const cv::Vec3d &p1, const cv::Vec3d &p2, const cv::Vec3d &p3, cv::Point2d &subpixel_pos )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double denominator = 2.0 * ( p1[ 2 ] - 2.0 * p2[ 2 ] + p3[ 2 ]);

        if ( -1e6 >- std::abs( denominator ) )
        {
            subpixel_pos = Point2d( p2[ 0 ], p2[ 1 ] );
        }
        else
        {
            double delta = ( p1[ 2 ] - p3[ 2 ] ) / denominator;

            double dx = (double)p3[ 0 ] - p2[ 0 ];
            double dy = (double)p3[ 1 ] - p2[ 1 ];

            subpixel_pos.x = p2[ 0 ] + delta * dx;
            subpixel_pos.y = p2[ 1 ] + delta * dy;
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::CalcSubPixel] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS OctoRefine::FindSubpixelFallingEdge( const cv::Mat &image, const cv::Point &pt1, const cv::Point &pt2,
                                               Point2d &subpixel_pos, double sigma )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        subpixel_pos = Point2d( -1.0, -1.0 );
        std::vector< cv::Point > coords;
        retVal = GetLineCoords( pt1, pt2, coords );
        if ( GC_OK == retVal )
        {
            if ( coords.size() < 3 )
            {
                subpixel_pos = cv::Point2d( static_cast< double >( pt1.x ), static_cast< double >( pt1.y ) );
            }
            else
            {
#ifdef OCTOREFINE_DEBUG
                Mat color;
                cvtColor( image, color, COLOR_GRAY2BGR );
                for ( size_t i = 0; i < coords.size(); ++i )
                {
                    color.at< cv::Vec3b >( coords[ i ].x, coords[ i ].y ) = cv::Vec3b( 255, 0, 0 );
                }
                imwrite( string( DEBUG_FOLDER ) + string( "050_search_line.png" ), color );
#endif
                std::vector<double> intensities;
                for (const auto& p : coords)
                {
                    if (p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows)
                    {
                        intensities.push_back( static_cast< double >( image.at< uchar >( p ) ) );
                    }
                    else
                    {
                        intensities.push_back( 0.0 );
                    }
                }
                cv::Mat src(intensities.size(), 1, CV_64F, intensities.data());
                cv::Mat smoothed;
                int ksize = std::max(3, int(6 * sigma + 1) | 1);
                cv::GaussianBlur(src, smoothed, cv::Size(ksize, 1), sigma, 0, cv::BORDER_REPLICATE);

                // 4. Compute gradient (centered difference, like np.gradient)
                std::vector< double > grad(intensities.size());
                for (int i = 1; i < smoothed.rows - 1; ++i)
                {
                    grad[i] = 0.5 * (smoothed.at< double >(i + 1, 0) - smoothed.at< double >(i - 1, 0));
                }
                grad[ 0 ] = smoothed.at<double>(1, 0) - smoothed.at<double>(0, 0);
                grad[grad.size() - 1] = smoothed.at< double >(smoothed.rows - 1, 0) - smoothed.at< double >(smoothed.rows - 2, 0);

                // 5. Find the strongest positive gradient (rising edge)
                size_t idx = 0;
                double min_val = grad[ 0 ];
                for ( size_t i = 0; i < grad.size(); ++i )
                {
                    if ( grad[ i ] > min_val )
                    {
                        idx = i;
                        min_val = grad[ i ];
                    }
                }

                if ( 1 > idx || coords.size() - 2 < idx )
                {
                    FILE_LOG( logERROR ) << "[OctoRefine::findSubpixelFallingEdge] Could not find edge position";
                    retVal = gc::GC_ERR;
                }
                else
                {
                    Vec3d p1( static_cast< double >( coords[ idx - 1 ].x ), static_cast< double >( coords[ idx - 1 ].y ), grad[ idx - 1 ] );
                    Vec3d p2( static_cast< double >( coords[ idx ].x ), static_cast< double >( coords[ idx ].y ), grad[ idx ] );
                    Vec3d p3( static_cast< double >( coords[ idx + 1 ].x ), static_cast< double >( coords[ idx + 1 ].y ), grad[ idx + 1 ] );
                    retVal = CalcSubPixel( p1, p2, p3, subpixel_pos );
                    if ( GC_OK == retVal )
                    {
#ifdef OCTOREFINE_DEBUG
                        color.at< cv::Vec3b >( subpixel_pos ) = cv::Vec3b( 0, 255, 255 );
                        color.at< cv::Vec3b >( pt1 ) = cv::Vec3b( 0, 255, 0 );
                        color.at< cv::Vec3b >( pt2 ) = cv::Vec3b( 0, 0, 255 );
                        imwrite( string( DEBUG_FOLDER ) + string( "020_edge_point.png" ), color );
#endif
                    }
                }
            }
        }
    }
    catch( const cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[OctoRefine::FindSubpixelFallingEdge] " << e.what();
        retVal = gc::GC_EXCEPT;
    }
    return retVal;
}
#else
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
            if ( coords.size() < 3 )
            {
                subpixel_pos = cv::Point2d(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
            }
            else
            {
#ifdef OCTOREFINE_DEBUG
                Mat color;
                cvtColor( image, color, COLOR_GRAY2BGR );
#endif
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
                cv::Mat src(intensities.size(), 1, CV_32F, intensities.data());
                cv::Mat smoothed;
                int ksize = std::max(3, int(6 * sigma + 1) | 1);
                cv::GaussianBlur(src, smoothed, cv::Size(ksize, 1), sigma, 0, cv::BORDER_REPLICATE);

                // 4. Compute gradient (centered difference, like np.gradient)
                std::vector<float> grad(intensities.size());
                for (int i = 1; i < smoothed.rows - 1; ++i)
                {
                    grad[i] = 0.5f * (smoothed.at<float>(i + 1, 0) - smoothed.at<float>(i - 1, 0));
                }
                grad[0] = smoothed.at<float>(1, 0) - smoothed.at<float>(0, 0);
                grad[grad.size() - 1] = smoothed.at<float>(smoothed.rows - 1, 0) - smoothed.at<float>(smoothed.rows - 2, 0);

                // 5. Find the strongest negative gradient (falling edge)
                auto min_it = std::min_element(grad.begin(), grad.end());
                int k = static_cast<int>(std::distance(grad.begin(), min_it));
                if (k <= 0 || k >= static_cast<int>(grad.size()) - 1)
                {
                    subpixel_pos = cv::Point2f(static_cast<float>(coords[k].x), static_cast<float>(coords[k].y));
                }
                else
                {
                    // 6. Parabolic subpixel interpolation
                    float y0 = grad[k - 1], y1 = grad[k], y2 = grad[k + 1];
                    float denom = 2.0f * (y0 - 2.0f * y1 + y2);
                    float subpixel_idx = static_cast<float>(k);
                    if (std::abs(denom) > 1e-6f)
                    {
                        subpixel_idx = k + (y0 - y2) / denom;
                    }

                    // 7. Map subpixel index to (x, y) along the line
                    float frac = subpixel_idx / (coords.size() - 1);
                    cv::Point2f pt1f(static_cast<float>(pt1.x), static_cast<float>(pt1.y));
                    cv::Point2f pt2f(static_cast<float>(pt2.x), static_cast<float>(pt2.y));
                    subpixel_pos = pt1f + (pt2f - pt1f) * frac;
#ifdef OCTOREFINE_DEBUG
                    color.at< cv::Vec3b >( subpixel_pos.x, subpixel_pos.y ) = cv::Vec3b( 0, 255, 255 );
                    color.at< cv::Vec3b >( pt1.x, pt1.y ) = cv::Vec3b( 0, 255, 0 );
                    color.at< cv::Vec3b >( pt2.x, pt2.y ) = cv::Vec3b( 0, 0, 255 );
#endif
                }
#ifdef OCTOREFINE_DEBUG
                imwrite( string( DEBUG_FOLDER ) + string( "020_edge_point.png" ), color );
#endif
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
#endif
gc::GC_STATUS OctoRefine::RefineFindExtend( const std::vector< cv::Point2d >& pts,
                                            std::vector < std::vector< std::pair< cv::Point, cv::Point > > > &extendedLines,
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
        double vxA = lineA.direction[0], vyA = lineA.direction[1];
        double x0A = lineA.point.x, y0A = lineA.point.y;
        double vxB = lineB.direction[0], vyB = lineB.direction[1];
        double x0B = lineB.point.x, y0B = lineB.point.y;
        cv::Point2d p0A(x0A, y0A), p0B(x0B, y0B);
        cv::Point2d b = p0B - p0A;
        cv::Matx22d M(vxA, -vxB, vyA, -vyB);
        float det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        if ( std::abs(det) < 1e-6f )
        {
            valid = false;
            FILE_LOG( logERROR ) << "[OctoRefine::RefineFindExtend] No viable conversion to type cv::Point2d";
            retVal = GC_ERR;
        }
        else
        {
            cv::Vec2d rhs(b.x, b.y);
            cv::Vec2d sol = M.inv() * rhs;
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
        if ( 8 != points.size() )
        {
            FILE_LOG( logERROR ) << "[OctoRefine::SortOctagonPoints] Invalid input point count. Must be 8.";
            retVal = gc::GC_ERR;
        }
        else
        {
            vector< Point2d > pts_sorted_x = points;
            std::sort( pts_sorted_x.begin(), pts_sorted_x.end(),  [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; } );
            int i0 = pts_sorted_x[ 0 ].x < pts_sorted_x[ 1 ].x ? 0 : 1;
            int i1 = 0 == i0 ? 1 : 0;
            int i4 = pts_sorted_x[ 6 ].x > pts_sorted_x[ 7 ].x ? 6 : 7;
            int i5 = 6 == i4 ? 7 : 6;

            vector< Point2d > pts_sorted_y = points;
            std::sort( pts_sorted_y.begin(), pts_sorted_y.end(),  [](const cv::Point& p1, const cv::Point& p2) { return p1.x < p2.x; } );
            int i2 = pts_sorted_y[ 6 ].y < pts_sorted_y[ 7 ].y ? 6 : 7;
            int i3 = 6 == i4 ? 7 : 6;
            int i6 = pts_sorted_y[ 0 ].y > pts_sorted_y[ 1 ].y ? 0 : 1;
            int i7 = 0 == i6 ? 1 : 0;

            ptsSorted.push_back( pts_sorted_x[ i0 ] );
            ptsSorted.push_back( pts_sorted_x[ i1 ] );
            ptsSorted.push_back( pts_sorted_y[ i2 ] );
            ptsSorted.push_back( pts_sorted_y[ i3 ] );
            ptsSorted.push_back( pts_sorted_x[ i4 ] );
            ptsSorted.push_back( pts_sorted_x[ i5 ] );
            ptsSorted.push_back( pts_sorted_y[ i6 ] );
            ptsSorted.push_back( pts_sorted_y[ i7 ] );
            // cout << ptsSorted << endl;
        }
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
