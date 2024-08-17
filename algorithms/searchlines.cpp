#include "log.h"
#include "searchlines.h"

using namespace cv;
using namespace std;

static double DISTANCE( Point2d a, Point2d b ) { return sqrt( ( b.x - a.x ) * ( b.x - a.x ) +
                                                              ( b.y - a.y ) * ( b.y - a.y ) ); }

namespace gc
{

SearchLines::SearchLines()
{

}
GC_STATUS SearchLines::CalcSearchLines( vector< Point > &searchLineCorners, std::vector< LineEnds > &searchLines )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( 4 != searchLineCorners.size() )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::CalcSearchLines] Invalid search line corner point count. Must be 4";
            retVal = GC_ERR;
        }
        else
        {
            sort( searchLineCorners.begin(), searchLineCorners.end(), []( Point const &a, Point const &b ) { return ( a.y < b.y ); } );

            const Point lftTop = searchLineCorners[ searchLineCorners[ 0 ].x > searchLineCorners[ 1 ].x ? 1 : 0 ];
            const Point rgtTop = searchLineCorners[ searchLineCorners[ 0 ].x > searchLineCorners[ 1 ].x ? 0 : 1 ];
            const Point lftBot = searchLineCorners[ searchLineCorners[ 2 ].x > searchLineCorners[ 3 ].x ? 3 : 2 ];
            const Point rgtBot = searchLineCorners[ searchLineCorners[ 2 ].x > searchLineCorners[ 3 ].x ? 2 : 3 ];

            searchLines.clear();
            searchLines.push_back( LineEnds( lftTop, lftBot ) );

            int widthTop = rgtTop.x - lftTop.x;
            int widthBot = rgtBot.x - lftBot.x;
            int width = std::max( widthTop, widthBot );
            double topInc = static_cast< double >( widthTop ) / static_cast< double >( width );
            double botInc = static_cast< double >( widthBot ) / static_cast< double >( width );

            double slopeTop, interceptTop;
            retVal = GetLineEquation( lftTop, rgtTop, slopeTop, interceptTop );
            if ( GC_OK == retVal )
            {
                double slopeBot, interceptBot;
                retVal = GetLineEquation( lftBot, rgtBot, slopeBot, interceptBot );
                if ( GC_OK == retVal )
                {
                    double topY, botY;
                    double topX = static_cast< double >( lftTop.x );
                    double botX = static_cast< double >( lftBot.x );

                    for ( int i = 0; i < width; ++i )
                    {
                        topX += topInc;
                        botX += botInc;
                        topY = slopeTop * topX + interceptTop;
                        botY = slopeBot * botX + interceptBot;
                        if ( MIN_SEARCH_LINE_LENGTH > DISTANCE( Point2d( topX, topY ), Point2d( botX, botY ) ) )
                        {
                            FILE_LOG( logERROR ) << "[CalibStopSign::CalcSearchLines] Search region not tall enough";
                            retVal = GC_ERR;
                            break;
                        }
                        if ( GC_ERR == retVal )
                        {
                            break;
                        }
                        searchLines.push_back( LineEnds( Point2d( topX, topY ), Point2d( botX, botY ) ) );
                    }
                    searchLines.push_back( LineEnds( rgtTop, rgtBot ) );
                }
            }
        }

        return retVal;
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[SearchLines::CalcSearchLines] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS SearchLines::GetLineEquation( const cv::Point2d pt1, const cv::Point2d pt2, double &slope, double &intercept )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        double deltaX = pt2.x - pt1.x;
        double deltaY = pt2.y - pt1.y;
        if ( numeric_limits< double >::epsilon() > deltaX )
        {
            FILE_LOG( logERROR ) << "[CalibStopSign::GetLineEquation] Invalid points: pt1 and pt2 cannot have the same value of X";
            retVal = GC_ERR;
        }
        else
        {
            slope = deltaY / deltaX;
            intercept = pt2.y - slope * pt2.x;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[SearchLines::GetLineEquation] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
