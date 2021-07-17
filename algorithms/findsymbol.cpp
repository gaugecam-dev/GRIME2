#include "log.h"
#include "findsymbol.h"
#include <opencv2/imgproc.hpp>

#ifdef DEBUG_FIND_CALIB_SYMBOL
#undef DEBUG_FIND_CALIB_SYMBOL
#include <opencv2/imgcodecs.hpp>
static const std::string DEBUG_RESULT_FOLDER = "/var/tmp/water/";
#endif

using namespace cv;

namespace gc
{

FindSymbol::FindSymbol()
{

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
GC_STATUS FindSymbol::FindCenter( const Mat &img, cv::Point2d &center )
{

}

} // namespace gc
