#include "log.h"
#include "findstaffgauge.h"
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

namespace gc
{

FindStaffGauge::FindStaffGauge()
{

}
GC_STATUS FindStaffGauge::FindTicks( const cv::Mat &img, STAFFGAUGE_TICK_TYPE tickType, std::vector<Point2d> &points )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( BLACK_BOTTOM_LEFT_CORNER == tickType )
        {
            retVal = CreateBlackBotLftCornerTemplates( img );
        }
        else if ( BLACK_TOP_RIGHT_POINT == tickType )
        {

        }
        else
        {
            FILE_LOG( logERROR ) << "[FindStaffGauge::FindTicks] Invalid tick type";
            retVal = GC_ERR;
        }
        if ( GC_OK == retVal )
        {
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindStaffGauge::FindTicks] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindStaffGauge::CreateBlackBotLftCornerTemplates( const cv::Mat &img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        int templateDimEven = 10 + ( 10 % 2 );

        int center = TICK_TEMPL_COUNT / 2;

        // create templates
        m_templates.clear();

        int tempDim = templateDimEven << 1;
        Mat matTemp( Size( tempDim, tempDim ), CV_8U );
        Mat matTempRot( Size( tempDim, tempDim ), CV_8U );

        for ( int i = 0; i < TICK_TEMPL_COUNT; ++i )
            m_templates.push_back( Mat( Size( templateDimEven, templateDimEven ), CV_8U ) );

        Mat templ = Mat::zeros( Size( 10, 10 ), CV_8UC1 );
        templ( Rect( 0, 5, 5, 5 ) ).setTo( 255 );

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

        // allocate template match space
        m_matchSpace.create( Size( img.cols - templateDimEven + 1,
                                   img.rows - templateDimEven + 1 ), CV_32F );
        m_matchSpaceSmall.create( Size( ( templateDimEven >> 1 ) + 1, ( templateDimEven >> 1 ) + 1 ), CV_32F );
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
        FILE_LOG( logERROR ) << "[FindCalibGrid::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} // namespace gc
