#include "log.h"
#include "findsymbol.h"
#include <opencv2/imgproc.hpp>

#ifndef DEBUG_FIND_CALIB_SYMBOL
#define DEBUG_FIND_CALIB_SYMBOL
#include <iostream>
#include <opencv2/imgcodecs.hpp>
static const std::string DEBUG_RESULT_FOLDER = "/var/tmp/water/";
#endif

static const double MIN_SYMBOL_CONTOUR_SIZE = 50;
static const double MIN_SYMBOL_CONTOUR_AREA = 1500;
static const int MIN_SYMBOL_CONTOUR_LENGTH = 7;
static const double MAX_SYMBOL_CONTOUR_ELONG = 1.5;
using namespace cv;
using namespace std;

namespace gc
{

static double elongation( Moments m );
static double EuclidianDistance( Point a, Point b );

FindSymbol::FindSymbol()
{

}
GC_STATUS FindSymbol::Find( const cv::Mat &img, std::vector< cv::Point > &symbolPoints )
{
    std::vector< SymbolCandidate > candidates;

    GC_STATUS retVal = FindRed( img, candidates );
    if ( GC_OK == retVal )
    {
        vector< Point > corners;
        for ( size_t i = 0; i < candidates.size(); ++i )
        {
            retVal = FindSymbolCorners( candidates[ i ].contour, corners );
            if ( GC_OK == retVal )
            {
#ifdef DEBUG_FIND_CALIB_SYMBOL
                Mat color;
                img.copyTo( color );
                for ( size_t j = 0; j < corners.size(); ++j )
                {
                    circle( color, corners[ j ], 25, Scalar( 255, 0, 0 ), 5 );
                    line( color, Point( corners[ j ].x - 25, corners[ j ].y ), Point( corners[ j ].x + 25, corners[ j ].y ), Scalar( 0, 255, 255 ), 1 );
                    line( color, Point( corners[ j ].x, corners[ j ].y - 25 ), Point( corners[ j ].x, corners[ j ].y + 25 ), Scalar( 0, 255, 255 ), 1 );
                }
                char msg[ 256 ];
                sprintf( msg, "%scorners_%03d.png", DEBUG_RESULT_FOLDER.c_str(), static_cast< int >( i ) );
                imwrite( msg, color );
#endif
            }
        }
    }

    return retVal;
}
GC_STATUS FindSymbol::FindRed( const cv::Mat &img, std::vector< SymbolCandidate > &symbolCandidates )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( img.type() != CV_8UC3 )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindRed] Image must be an 8-bit BGR image to find red";
            retVal = GC_ERR;
        }
        else
        {
            Mat3b hsv;
            cvtColor( img, hsv, COLOR_BGR2HSV );

            Mat1b mask1, mask2;
            inRange( hsv, Scalar( 0, 70, 50 ), Scalar( 10, 255, 255 ), mask1 );
            inRange( hsv, Scalar( 170, 70, 50 ), Scalar( 180, 255, 255 ), mask2 );

            Mat1b redMask = mask1 | mask2;

            vector< vector< Point > > contours;
            findContours( redMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

#ifdef DEBUG_FIND_CALIB_SYMBOL
            Mat color;
            img.copyTo( color );
            imwrite( DEBUG_RESULT_FOLDER + "red_mask.png", redMask );
#endif

            Moments m;
            double area, elong;
            symbolCandidates.clear();
            for ( size_t i = 0; i < contours.size(); ++i )
            {
                if ( MIN_SYMBOL_CONTOUR_LENGTH <= contours[ i ].size() )
                {
                    area = contourArea( contours[ i ] );
                    if ( MIN_SYMBOL_CONTOUR_AREA <= area )
                    {
                        m = moments( contours[ i ] );
                        elong = elongation( m );
                        cout << "elongation=" << elong << endl;
                        if ( MAX_SYMBOL_CONTOUR_ELONG >= elong )
                        {
                            symbolCandidates.push_back( SymbolCandidate( contours[ i ], area, elong ) );
#ifdef DEBUG_FIND_CALIB_SYMBOL
                            drawContours( color, contours, i, Scalar( 0, 255, 255 ), 3 );
#endif
                        }
                    }
                }
            }
            if ( symbolCandidates.empty() )
            {
                FILE_LOG( logERROR ) << "[FindSymbol::FindRed] No symbol candidates found";
                retVal = GC_ERR;
            }
#ifdef DEBUG_FIND_CALIB_SYMBOL
            else
            {
                imwrite( DEBUG_RESULT_FOLDER + "candidates.png", color );
            }
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::FindRed] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::FindSymbolCorners( const std::vector< cv::Point > &contour, std::vector< cv::Point > &corners )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( contour.size() < MIN_SYMBOL_CONTOUR_SIZE )
        {
            FILE_LOG( logERROR ) << "[FindSymbol::FindSymbolCorners] Contour must have at least " << MIN_SYMBOL_CONTOUR_SIZE << " contour points";
            retVal = GC_ERR;
        }
        else
        {
            RotatedRect rotRect = fitEllipse( contour );

            double spokeLength;
            int maxSpokeIdx = -1;
            double maxSpokeLength = -1.0;
            vector< double > spokeLengths;
            for ( size_t i = 0; i < contour.size(); ++i )
            {
                spokeLength = EuclidianDistance( contour[ i ], rotRect.center );
                if ( maxSpokeLength < spokeLength )
                {
                    maxSpokeLength = spokeLength;
                    maxSpokeIdx = static_cast< int >( i );
                }
                spokeLengths.push_back( spokeLength );
            }

            corners.push_back( contour[ maxSpokeIdx ] );
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::FindSymbolCorners] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::FindCenter( const Mat &img, SymbolMatch &matchResult )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        Mat searchImg;
        Mat matchSpace;
        double dMin, dMax;
        Point ptMin, ptMax;
        double sizeRatio = static_cast< double >( img.rows ) / static_cast< double >( img.cols );
        for ( int w = SYMBOL_SEARCH_IMAGE_WIDTH_START; w <= SYMBOL_SEARCH_IMAGE_WIDTH_END; w += SYMBOL_SEARCH_IMAGE_WIDTH_INC )
        {
            resize( img, searchImg, Size( w, cvRound( static_cast< double >( w ) * sizeRatio ) ), 0.0, 0.0, INTER_CUBIC );
            for ( size_t i = 0; i < symbolTemplates.size(); ++i )
            {
                matchSpace = Mat();
                matchTemplate( searchImg, symbolTemplates[ i ], matchSpace, cv::TM_CCOEFF_NORMED );
                minMaxLoc( matchSpace, &dMin, &dMax, &ptMin, &ptMax );
                if ( dMax >= matchResult.score )
                {
                    ptMax.x += symbolTemplates[ i ].cols / 2;
                    ptMax.y += symbolTemplates[ i ].rows / 2;
                    matchResult = SymbolMatch( ptMax, dMax, w, i );
                }
            }
        }
        double scale = static_cast< double >( img.cols ) / static_cast< double >( matchResult.width );
        matchResult.pt = Point( cvRound( static_cast< double >( matchResult.pt.x ) * scale ), cvRound( static_cast< double >( matchResult.pt.y ) *scale ) );

#ifdef DEBUG_FIND_CALIB_SYMBOL   // debug of template rotation
        std::cout << "Score=" << matchResult.score << " x=" << matchResult.pt.x << " y=" << matchResult.pt.y;
        std::cout << " angleIndex=" << matchResult.angleIdx << " scale=" << scale << std::endl;
        Mat color;
        cvtColor( img, color, COLOR_GRAY2BGR );
        circle( color, matchResult.pt, 25, Scalar( 255, 0, 0 ), 5 );
        line( color, Point( matchResult.pt.x - 25, matchResult.pt.y ), Point( matchResult.pt.x + 25, matchResult.pt.y ), Scalar( 0, 255, 255 ), 3 );
        line( color, Point( matchResult.pt.x, matchResult.pt.y - 25 ), Point( matchResult.pt.x, matchResult.pt.y + 25 ), Scalar( 0, 255, 255 ), 3 );
        imwrite( DEBUG_RESULT_FOLDER + "sign_center.png", color );
#endif
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::FindCenter] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::CreateSymbolTemplates( const cv::Mat &refTemplate )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        // create templates
        symbolTemplates.clear();

        Mat refAdj;
        double sizeRatio = static_cast< double >( SYMBOL_TEMPL_WIDTH ) / static_cast< double >( refTemplate.cols );
        Size sizeAdj( SYMBOL_TEMPL_WIDTH, cvRound( sizeRatio * static_cast< double >( refTemplate.rows ) ) );
        resize( refTemplate, refAdj, sizeAdj, 0.0, 0.0, INTER_CUBIC );

        int templateCount = cvRound( 2.0 * ( SYMBOL_TEMPL_ANGLE_MAX / SYMBOL_TEMPL_ANGLE_INC ) ) + 1;

        int tempWidth = refAdj.cols << 1;
        int tempHeight = refAdj.rows << 1;
        Rect roiRotate( tempWidth >> 2, tempHeight >> 2, refAdj.cols, refAdj.rows );
        Mat matTemp = Mat::zeros( Size( tempWidth, tempHeight ), CV_8U );
        refAdj.copyTo( matTemp( roiRotate ) );
        Mat matTempRot( Size( tempWidth, tempHeight ), CV_8U );

        for ( int i = 0; i < templateCount; ++i )
            symbolTemplates.push_back( Mat::zeros( sizeAdj, CV_8U ) );

        // create the rotated templates
        int center = templateCount / 2;

        Mat matTemplateTemp = matTemp( roiRotate );

        matTemplateTemp.copyTo( symbolTemplates[ static_cast< size_t >( center ) ] );
#ifdef DEBUG_FIND_CALIB_SYMBOL   // debug of template rotation
        char msg[ 1024 ];
        sprintf( msg, "%02d_template_center.png", center );
        imwrite( DEBUG_RESULT_FOLDER + msg, symbolTemplates[ static_cast< size_t >( center ) ] );
        imwrite( DEBUG_RESULT_FOLDER + "ref_template.png", refTemplate );
#endif

        for ( int i = 0; i < center; ++i )
        {
            retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i - center ) );
            if ( GC_OK != retVal )
                break;

            matTemplateTemp = matTempRot( roiRotate );
            matTemplateTemp.copyTo( symbolTemplates[ static_cast< size_t >( i ) ] );

#ifdef DEBUG_FIND_CALIB_SYMBOL   // debug of template rotation
            sprintf( msg, "%02d_template.png", i );
            imwrite( DEBUG_RESULT_FOLDER + msg, symbolTemplates[ static_cast< size_t >( i ) ] );
#endif
            retVal = RotateImage( matTemp, matTempRot, static_cast< double >( i + 1 ) );
            if ( 0 != retVal )
                break;

            matTemplateTemp = matTempRot( roiRotate );
            matTemplateTemp.copyTo( symbolTemplates[ static_cast< size_t >( center + i + 1 ) ] );
#ifdef DEBUG_FIND_CALIB_SYMBOL   // debug of template rotation
            sprintf( msg, "%02d_template.png", center + i + 1 );
            imwrite( DEBUG_RESULT_FOLDER + msg, symbolTemplates[ static_cast< size_t >( center + i + 1 ) ] );
#endif
        }
    }
    catch( cv::Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindSymbol::CreateSymbolTemplates] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindSymbol::RotateImage( const Mat &src, Mat &dst, const double angle )
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
        FILE_LOG( logERROR ) << "[RotateImage::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

// helper functions
static double EuclidianDistance( Point a, Point b )
{
    return sqrt( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) );
}
static double elongation( Moments m )
{
    double x = m.mu20 + m.mu02;
    double y = 4 * m.mu11 * m.mu11 + ( m.mu20 - m.mu02 ) * ( m.mu20 - m.mu02 );
    double srt = sqrt( y );
    if ( x - srt > DBL_EPSILON )
        return ( x + srt ) / ( x - srt );
    else
        return 1.0;
}

} // namespace gc
