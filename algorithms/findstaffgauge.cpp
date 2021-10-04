#include "log.h"
#include "findstaffgauge.h"
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#ifndef __DEBUG_STAFFGAUGE
#define __DEBUG_STAFFGAUGE
#define SET_DEBUG_FOLDER
#endif

#ifdef __DEBUG_STAFFGAUGE_TEMPLATES
#undef __DEBUG_STAFFGAUGE_TEMPLATES
#undef SET_DEBUG_FOLDER
#endif


#ifdef SET_DEBUG_FOLDER
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
using namespace boost;
namespace fs = filesystem;
#ifdef WIN32
static const string "c:/gaugecam/debug"
#else
static const string DEBUG_FOLDER = "/var/tmp/water/";
#endif
#endif

static double DistToLine( const cv::Point2d pt, const cv::Point2d lnPt1, const cv::Point2d lnPt2 )
{
    double dNum = fabs( ( lnPt2.x - lnPt1.x ) * ( lnPt1.y - pt.y ) - ( lnPt1.x - pt.x ) * ( lnPt2.y - lnPt1.y ) );
    double dDenom = sqrt( ( lnPt2.x - lnPt1.x ) * ( lnPt2.x - lnPt1.x ) + ( lnPt2.y - lnPt1.y ) * ( lnPt2.y - lnPt1.y ) );
    return 0.0 == dDenom ? 0.0 : dNum / dDenom;
}

namespace gc
{

FindStaffGauge::FindStaffGauge()
{
    if ( !fs::exists( DEBUG_FOLDER ) )
    {
        bool bRet = fs::create_directories( DEBUG_FOLDER );
        if ( !bRet )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::FindStaffGauge] Could not create debug folder " << DEBUG_FOLDER;
        }
    }
}
GC_STATUS FindStaffGauge::FindTicks( const cv::Mat &img, StaffGaugeTickType tickType )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        retVal = CreateTemplates( img, tickType );
        if ( GC_OK == retVal )
        {
            int numToFind = -1;
            if ( BLACK_BOTTOM_LEFT_CORNER == tickType )
                numToFind = TICK_TARGET_COUNT_LEFT;
            else if ( BLACK_TOP_RIGHT_POINT == tickType )
                numToFind = TICK_TARGET_COUNT_RIGHT;
            retVal = FindTemplates( img, TICK_TEMPL_MATCH_MIN_SCORE, numToFind );
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::FindTicks] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::Find( const cv::Mat &img, cv::Point2d ptTopTickPos, const double distTickToTick,
                                const std::vector< double > tickLengths )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat img8u;
        if ( img.type() == CV_8UC3 )
            cvtColor( img, img8u, COLOR_BGR2GRAY );
        else
            img8u = img;

        retVal = FindTicks( img8u, BLACK_TOP_RIGHT_POINT );
        if ( GC_OK == retVal )
        {
            retVal = CalcWorldPts( img8u, ptTopTickPos, distTickToTick, tickLengths );
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::Find] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::CalcWorldPts( const cv::Mat &img, cv::Point2d ptTopTickPos, const double distTickToTick,
                                        const std::vector< double > tickLengthsLowToHigh )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( TICK_POINT_COUNT_MIN > m_matchItems.size() )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::CalcWorldPts] Too few tick points found";
            retVal = GC_ERR;
        }
        else
        {
            Point2d pt;
            vector< Point2d > intersectPts;
            for ( size_t i = 0; i < m_matchItems.size(); ++i )
            {

            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::CalcWorldPts] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::CreateTemplates( const cv::Mat &img, const StaffGaugeTickType tickType )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat matTemp;
        int tempDim, templateDimEven, templateDim = -1;
        if ( BLACK_BOTTOM_LEFT_CORNER == tickType )
        {
            templateDim = 20;
            templateDimEven = templateDim + ( templateDim % 2 );
            tempDim = templateDimEven << 1;
            matTemp = Mat::zeros( Size( tempDim, tempDim ), CV_8UC1 );
            matTemp( Rect( tempDim >> 1, tempDim >> 1, tempDim >> 1, tempDim >> 1 ) ).setTo( 255 );
        }
        else if ( BLACK_TOP_RIGHT_POINT == tickType )
        {
            templateDim = 20;
            templateDimEven = templateDim + ( templateDim % 2 );
            tempDim = templateDimEven << 1;
            matTemp = Mat::zeros( Size( tempDim, tempDim ), CV_8UC1 );
            Mat matDraw = matTemp( Rect( tempDim >> 2, tempDim >> 2, tempDim >> 1, tempDim >> 1 ) );
            vector< Point > contour = { Point( 0, matDraw.rows >> 1 ),
                                        Point( matDraw.cols >> 1, matDraw.rows >> 1 ),
                                        Point( 0, matDraw.rows - 1 ),
                                        Point( 0, matDraw.rows >> 1 ) };
            drawContours( matDraw, vector< vector< Point > >( 1, contour ), -1, Scalar( 255 ), FILLED );
            matTemp = ~matTemp;
#ifdef __DEBUG_STAFFGAUGE_TEMPLATE
            imwrite( "/var/tmp/water/base_templ.png", matTemp );
#endif
        }
        else
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::CreateTemplates] Invalid tick type";
            retVal = GC_ERR;
        }

        if ( GC_OK == retVal )
        {
            int center = TICK_TEMPL_COUNT / 2;

            // create templates
            m_templates.clear();

            Mat matTempRot( Size( tempDim, tempDim ), CV_8U );

            for ( int i = 0; i < TICK_TEMPL_COUNT; ++i )
                m_templates.push_back( Mat( Size( templateDimEven, templateDimEven ), CV_8U ) );


            // create the rotated templates
            Rect roiRotate( templateDimEven >> 1, templateDimEven >> 1, templateDimEven, templateDimEven );
            Mat matTemplateTemp = matTemp( roiRotate );
            matTemplateTemp.copyTo( m_templates[ static_cast< size_t >( center ) ] );

            for ( int i = 0; i < center; ++i )
            {
                retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i - center ) );
                if ( GC_OK != retVal )
                    break;

                matTemplateTemp = matTempRot( roiRotate );
                matTemplateTemp.copyTo( m_templates[ static_cast< size_t >( i ) ] );

                retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i + 1 ) );
                if ( 0 != retVal )
                    break;

                matTemplateTemp = matTempRot( roiRotate );
                matTemplateTemp.copyTo( m_templates[ static_cast< size_t >( center + i + 1 ) ] );
            }
#ifdef __DEBUG_STAFFGAUGE_TEMPLATES
            for ( size_t i = 0; i < m_templates.size(); ++i )
                imwrite( DEBUG_FOLDER + "template_" + to_string( i ) + ".png", m_templates[ i ] );
#endif

            // allocate template match space
            m_matchSpace = Mat();
            m_matchSpaceSmall = Mat();
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::CreateBlackBotLftCornerTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::RotateImage( const Mat &src, Mat &dst, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Point2d ptCenter = Point2d( static_cast< double >( src.cols ) / 2.0, static_cast< double >( src.rows ) / 2.0 );
        Mat matRotMatrix = getRotationMatrix2D( ptCenter, angle, 1.0 );
        warpAffine( src, dst, matRotMatrix, dst.size(), INTER_CUBIC );
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindStaffGauge::FindTemplates( const Mat &img, const double minScore, const int targetCount, const string resultFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( m_templates.empty() )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::FindTemplates] Templates not devined";
            retVal = GC_ERR;
        }
        else if ( img.empty() )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::FindTemplates] Cannot find targets in a NULL image";
            retVal = GC_ERR;
        }
        if ( 0.01 > minScore || 1.0 < minScore )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::FindTemplates] Invalid minimum target score " << minScore;
            retVal = GC_ERR;
        }
        else
        {
            retVal = MatchTemplate( m_templates.size() >> 1, img, minScore, targetCount * 2 );
            if ( GC_OK == retVal )
            {
                vector< TickItem > itemsTemp;
                for ( size_t i = 0; i < m_matchItems.size(); ++i )
                    itemsTemp.push_back( m_matchItems[ i ] );

                m_matchItems.clear();
                for ( size_t i = 0; i < itemsTemp.size(); ++i )
                {
                    for ( size_t j = 0; j < m_templates.size(); ++j )
                    {
                        retVal = MatchRefine( static_cast< int >( j ), img, minScore, 1, itemsTemp[ i ] );
                        if ( GC_OK != retVal )
                            break;
                    }
                    if ( GC_OK != retVal )
                        break;
                    m_matchItems.push_back( itemsTemp[ i ] );
                }

                if ( TICK_POINT_COUNT_MIN > m_matchItems.size() )
                {
                    FILE_LOG( logERROR ) << "[FindStaffGauge::FindTemplates] Ticks found=" << m_matchItems.size()
                                         << ". Should be at least " << TICK_POINT_COUNT_MIN;
                    retVal = GC_ERR;
                }
                else
                {
                    pixelPts.clear();
                    for ( size_t i = 0; i < m_matchItems.size(); ++i )
                    {
                        pixelPts.push_back( m_matchItems[ i ].pt );
                    }
                    retVal = FindTickTipLine( img, pixelPts, linePt1, linePt2 );
                    if ( GC_OK == retVal )
                    {
                        for ( size_t i = 0; i < m_matchItems.size(); ++i )
                        {
                            m_matchItems[ i ].x_length = DistToLine( m_matchItems[ i ].pt, linePt1, linePt2 );
                        }
                    }

                    if ( !resultFilepath.empty() )
                    {
                        Mat color;
                        cvtColor( img, color, COLOR_GRAY2BGR );
                        for ( size_t i = 0; i < m_matchItems.size(); ++i )
                        {
                            line( color, Point( m_matchItems[ i ].pt.x - 5, m_matchItems[ i ].pt.y ),
                                    Point( m_matchItems[ i ].pt.x + 5, m_matchItems[ i ].pt.y ), Scalar( 0, 0, 255 ) );
                            line( color, Point( m_matchItems[ i ].pt.x,  m_matchItems[ i ].pt.y - 5 ),
                                    Point( m_matchItems[ i ].pt.x, m_matchItems[ i ].pt.y + 5 ), Scalar( 0, 0, 255 ) );
                        }
                        if ( GC_OK == retVal )
                        {
                            line( color, linePt1, linePt2, Scalar( 0, 255, 255 ), 1 );
                        }

                        bool isOK = imwrite( resultFilepath, color );
                        if ( !isOK )
                        {
                            FILE_LOG( logERROR ) << "[" << __func__ << "[FindStaffGauge::FindTargets]"
                                                                       " Could not save result calib grid find to cache";
                            retVal = GC_ERR;
                        }
                    }
#ifdef DEBUG_FIND_CALIB_GRID   // output template matches to CSV file
                    FILE *fp = fopen( ( DEBUG_RESULT_FOLDER + "matches.csv" ).c_str(), "w" );
                    if ( nullptr != fp )
                    {
                        fprintf( fp, "Score, X, Y\n" );
                        for ( size_t i = 0; i < m_matchItems.size(); ++i )
                        {
                            fprintf( fp, "%.3f, %.3f, %.3f\n", m_matchItems[ i ].score,
                                     m_matchItems[ i ].pt.x, m_matchItems[ i ].pt.y );
                        }
                        fclose( fp );
                    }
#endif
                }
            }
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindStaffGauge::MatchRefine( const int index, const cv::Mat &img, const double minScore,
                                       const int numToFind, TickItem &item )
{
    GC_STATUS retVal = GC_OK;

    if ( 0 > index || static_cast< int >( m_templates.size() ) <= index )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchRefine]"
                                                   " Attempted to find template index=" << index << \
                                                   " Must be in range 0-" << m_templates.size() - 1;
        retVal = GC_ERR;
    }
    else if ( 0.05 > minScore || 1.0 < minScore )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchRefine]  Min score %.3f must be in range 0.05-1.0" << minScore;
        retVal = GC_ERR;
    }
    else if ( 1 > numToFind || 1000 < numToFind )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchRefine]"
                                                   " Attempted to find " << numToFind << \
                                                   " matches.  Must be in range 1-1000";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            Rect rect;
            double minScore, maxScore;
            Point ptMin, ptMax;
            Point2d ptFinal;

            m_matchSpace = 0;
            TickItem itemTemp;

            matchTemplate( img, m_templates[ static_cast< size_t >( index ) ], m_matchSpace, TM_CCOEFF_NORMED );
#ifdef DEBUG_FIND_CALIB_GRID
            Mat matTemp;
            normalize( m_matchSpace, matTemp, 255.0 );
            imwrite( DEBUG_RESULT_FOLDER + "bowtie_match_fine.png", matTemp );
#endif
            rect.x = std::max( 0, cvRound( item.pt.x )- ( m_templates[ 0 ].cols >> 1 ) - ( m_templates[ 0 ].cols >> 2 ) );
            rect.y = std::max( 0, cvRound( item.pt.y ) - ( m_templates[ 0 ].rows >> 1 ) - ( m_templates[ 0 ].rows >> 2 ) );
            rect.width = m_templates[ 0 ].cols + ( m_templates[ 0 ].cols >> 1 );
            rect.height = m_templates[ 0 ].rows + ( m_templates[ 0 ].rows >> 1 );
            if ( rect.x + rect.width >= img.cols )
                rect.x = img.cols - rect.width;
            if ( rect.y + rect.height >= img.rows )
                rect.y = img.rows - rect.height;

            Mat matROI = img( rect );
            m_matchSpaceSmall = 0;

            matchTemplate( matROI, m_templates[ static_cast< size_t >( index ) ], m_matchSpaceSmall, TM_CCOEFF_NORMED );
            minMaxLoc( m_matchSpaceSmall, &minScore, &maxScore, &ptMin, &ptMax );
            if ( maxScore > item.score )
            {
                retVal = SubpixelPointRefine( m_matchSpaceSmall, ptMax, ptFinal );
                if ( GC_OK == retVal )
                {
                    // ptFinal = Point2d( static_cast< double >( ptMax.x ), static_cast< double >( ptMax.y ) );
                    item.score = maxScore;
                    item.pt.x = static_cast< double >( rect.x ) + ptFinal.x + static_cast< double >( m_templates[ 0 ].cols ) / 2.0;
                    item.pt.y = static_cast< double >( rect.y ) + ptFinal.y + static_cast< double >( m_templates[ 0 ].rows ) / 2.0;
                }
                else
                {
                    item.score = 0.0;
                    item.pt.x = static_cast< double >( rect.x ) + ptMax.x + static_cast< double >( m_templates[ 0 ].cols ) / 2.0;
                    item.pt.y = static_cast< double >( rect.y ) + ptMax.y + static_cast< double >( m_templates[ 0 ].rows ) / 2.0;
                    retVal = GC_OK;
                }
            }
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::MatchRefine] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindStaffGauge::MatchTemplate( const int index, const Mat &img, const double minScore, const int numToFind )
{
    GC_STATUS retVal = GC_OK;

    if ( 0 > index || static_cast< int >( m_templates.size() ) <= index )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchTemplate]"
                                " Attempted to find template index=" << index << \
                                " Must be in range 0-" << m_templates.size() - 1;
        retVal = GC_ERR;
    }
    else if ( 0.05 > minScore || 1.0 < minScore )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchTemplate]"
                                " Min score %.3f must be in range 0.05-1.0" << minScore;
        retVal = GC_ERR;
    }
    else if ( 1 > numToFind || 1000 < numToFind )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::MatchTemplate]"
                                                   " Attempted to find " << numToFind << \
                                                   " matches.  Must be in range 1-1000";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            Rect rect;
            double dMin, dMax;
            Point ptMin, ptMax;
            Point2d ptFinal;
            TickItem itemTemp;

            m_matchSpace = 0.0;
            m_matchItems.clear();
            matchTemplate( img, m_templates[ static_cast< size_t >( index ) ], m_matchSpace, cv::TM_CCOEFF_NORMED );

            // TODO: Debug here for find move target FAIL
#ifdef DEBUG_FIND_CALIB_GRID
            Mat matTemp;
            normalize( m_matchSpace, matTemp, 255.0 );
            imwrite( DEBUG_RESULT_FOLDER + "bowtie_match_original.png", img );
            imwrite( DEBUG_RESULT_FOLDER + "bowtie_match_coarse.png", matTemp );
            imwrite( DEBUG_RESULT_FOLDER + "bowtie_match_coarse_double.tiff", m_matchSpace );
#endif

            for ( int i = 0; i < numToFind; i++ )
            {
                minMaxLoc( m_matchSpace, &dMin, &dMax, &ptMin, &ptMax );
                if (  0 < ptMax.x && 0 < ptMax.y && img.cols - 1 > ptMax.x && img.rows - 1 > ptMax.y )
                {
                    if ( dMax >= minScore )
                    {
                        itemTemp.score = dMax;
                        itemTemp.pt.x = static_cast< double >( ptMax.x ) + static_cast< double >( m_templates[ 0 ].cols ) / 2.0;
                        itemTemp.pt.y = static_cast< double >( ptMax.y ) + static_cast< double >( m_templates[ 0 ].rows ) / 2.0;
                        m_matchItems.push_back( itemTemp );
                    }
                    else
                        break;
                }
                circle( m_matchSpace, ptMax, 17, Scalar( 0.0 ), FILLED );
            }
            if ( m_matchItems.empty() )
            {
                FILE_LOG( logERROR ) << "[FindStaffGauge::MatchTemplate] No template matches found";
                retVal = GC_ERR;
            }
            else
            {
                sort( m_matchItems.begin(), m_matchItems.end(), []( const TickItem &a, const TickItem &b ) { return a.pt.y < b.pt.y; } );
                for ( size_t i = 1; i < m_matchItems.size(); ++i )
                {
                    m_matchItems[ i ].y_interval = m_matchItems[ i ].pt.y - m_matchItems[ i - 1 ].pt.y;
                }
            }
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::MatchTemplate] " << e.what();
            return GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindStaffGauge::SubpixelPointRefine( const Mat &matchSpace, const Point ptMax, Point2d &ptResult )
{
    GC_STATUS retVal = GC_OK;
    if ( 1 > ptMax.x || 1 > ptMax.y || matchSpace.cols - 2 < ptMax.x || matchSpace.rows - 2 < ptMax.y )
    {
#ifdef DEBUG_FIND_CALIB_GRID
        FILE_LOG( logWARNING ) << "[FindStaffGauge::SubpixelPointRefine] Invalid point (not on image) for subpixel refinement";
#endif
        retVal = GC_WARN;
    }
    else if ( CV_32FC1 != matchSpace.type() )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::SubpixelPointRefine] Invalid image format for subpixel refinement";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            float val;
            int col, row;
            float total = 0.0f;
            float totalX = 0.0f;
            float totalY = 0.0f;
            for ( row = ptMax.y - 1; row < ptMax.y + 2; ++row )
            {
                for ( col = ptMax.x - 1; col < ptMax.x + 2; ++col )
                {
                    val = matchSpace.at< float >( Point( col, row ) );
                    total += val;
                    totalX += val * static_cast< float >( col );
                    totalY += val * static_cast< float >( row );
                }
            }
            ptResult = Point2d( static_cast< double >( totalX / total ),
                                static_cast< double >( totalY / total ) );
        }
        catch( Exception &e )
        {
            FILE_LOG( logERROR ) << "[FindCalibGrid::SubpixelPointRefine] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS FindStaffGauge::FindTickTipLine( const cv::Mat &img, const std::vector< cv::Point2d > pts, cv::Point2d &pt1, cv::Point2d &pt2 )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Vec4d lne;
        fitLine( pts, lne, DIST_L12, 0.0, 0.01, 0.01 );

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // line equation: y = mx + b
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Point2d pt1x0, pt1y0, pt2x0, pt2y0;

        double a = lne[ 1 ];
        double b = -lne[ 0 ];
        double c = lne[ 0 ] * lne[ 3 ] - lne[ 1 ] * lne[ 2 ];

        double denom = ( 0.0 == a ? std::numeric_limits< double >::epsilon() : a );
        pt1y0.y = 0.0;
        pt1y0.x = c / -denom;
        pt2y0.y = static_cast< double >( img.rows - 1 );
        pt2y0.x = ( b * pt2y0.y + c ) / -denom;

        denom = ( 0.0 == b ? std::numeric_limits< double >::epsilon() : b );
        pt1x0.x = 0.0;
        pt1x0.y = c / -denom;
        pt2x0.x = static_cast< double >( img.cols - 1 );
        pt2x0.y = ( a * pt2x0.x + c ) / -denom;

        if ( 0.0 <= pt1y0.x && 0.0 <= pt1y0.y && img.cols > pt1y0.x && img.rows > pt1y0.y )
        {
            pt1 = pt1y0;
        }
        else
        {
            pt1 = pt1x0;
        }

        if ( 0.0 <= pt2y0.x && 0.0 <= pt2y0.y && img.cols > pt2y0.x && img.rows > pt2y0.y )
        {
            pt2 = pt2y0;
        }
        else
        {
            pt2 = pt2x0;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindCalibGrid::FindTickTipLine] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::GetClosestPointOnLine( const cv::Point2d linePt1, const cv::Point2d linePt2,
                                                 const cv::Point2d pt, cv::Point &ptOnLine )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double APx = pt.x - linePt1.x;
        double APy = pt.y - linePt1.y;
        double ABx = linePt2.x - linePt1.x;
        double ABy = linePt2.y - linePt1.y;
        double magAB2 = ABx * ABx + ABy * ABy;
        double ABdotAP = ABx * APx + ABy * APy;
        double t = ABdotAP / magAB2;

        if ( 0 > t )
        {
            ptOnLine = linePt1;
        }
        else if ( 1 < t )
        {
            ptOnLine = linePt2;
        }
        else
        {
            ptOnLine.x = linePt1.x + ABx * t;
            ptOnLine.y = linePt2.y + ABy * t;
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindCalibGrid::FindTickTipLine] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
