/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "guivisapp.h"
#include <mutex>
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <filesystem>
#include "../algorithms/log.h"
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "../algorithms/timestampconvert.h"
#include "../algorithms/kalman.h"
#include "../algorithms/wincmd.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = std::filesystem;

#ifdef _WIN32
static const char LOG_FILE_FOLDER[] = "c:/temp/gaugecam/";
#else
static const char LOG_FILE_FOLDER[] = "/var/tmp/gaugecam/";
#endif

using namespace cv;
using namespace std;

std::mutex mtx_img;

namespace gc
{

GuiVisApp::GuiVisApp() :
    m_isRunning( false ),
    m_threadType( NONE_RUNNING ),
    m_bShowRuler( false ),
    m_strConfigFolder( "./config" ),
    m_strCurrentImageFilepath( "" ),
    m_pFileLog( nullptr )
{
    fs::path p( LOG_FILE_FOLDER );
    bool folderExists = fs::exists( p );
    if ( !folderExists )
        folderExists = fs::create_directories( p );

    char buf[ 256 ];
    sprintf( buf, "%sgrime.log", folderExists ? LOG_FILE_FOLDER : "" );

#if WIN32
    fopen_s( &m_pFileLog, buf, "w" );
#else
    m_pFileLog = fopen( buf, "w" );
#endif
    Output2FILE::Stream() = m_pFileLog;
    // Output2FILE::Stream() = stdout;

}
GuiVisApp::~GuiVisApp() { Destroy(); }
GC_STATUS GuiVisApp::Init( const string strConfigFolder, Size &sizeImg )
{
    GC_STATUS retVal = GC_OK;
    if ( GC_OK == retVal )
    {
        retVal = InitBuffers( sizeImg );
        if ( 0 == retVal )
        {
            m_strConfigFolder = strConfigFolder;
            retVal = ReadSettings( strConfigFolder );
            if ( GC_OK <= retVal )
            {
                retVal = InitBuffers( sizeImg );
            }
        }
    }

    return retVal;
}
GC_STATUS GuiVisApp::InitBuffers( const Size sizeImg )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        m_matGray.create( sizeImg, CV_8UC1 );
        m_matColor.create( sizeImg, CV_8UC3 );
        m_matDisplay.create( sizeImg, CV_8UC3 );
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::Destroy()
{
    GC_STATUS retVal = GC_OK;
    return retVal;
}
bool GuiVisApp::IsInitialized()
{
    return ( m_matColor.empty() || m_matGray.empty() ) ? false : true;
}
GC_STATUS GuiVisApp::GetImage(const Size sizeImg, const size_t nStride, const int nType,
                               uchar *pPix, const IMG_BUFFERS nImgColor, const IMG_DISPLAY_OVERLAYS overlays )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        retVal = GetImageOverlay( nImgColor, overlays );
        if ( GC_OK != retVal )
        {
            FILE_LOG( logWARNING ) << "Could not perform GetImageOverlay()";
            m_matDisplay.setTo( 0 );
        }
        else
        {
            retVal = GetImageColor( m_matDisplay, sizeImg, nStride, nType, pPix, false );
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::GetImageOverlay( const IMG_BUFFERS nImgColor, const IMG_DISPLAY_OVERLAYS overlays )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( BUF_GRAY == nImgColor )
        {
            cvtColor( m_matGray, m_matDisplay, COLOR_GRAY2BGR );
        }
        else if ( BUF_RGB == nImgColor )
        {
            m_matDisplay = m_matColor.clone();
        }
        else if ( BUF_OVERLAY == nImgColor )
        {
            bool hasCalib = false;
            Mat matTemp = m_matColor.clone();
            if ( ( overlays & CALIB_SCALE ) ||
                 ( overlays & CALIB_GRID ) ||
                 ( overlays & MOVE_ROIS ) ||
                 ( overlays & SEARCH_ROI ) ||
                 ( overlays & TARGET_ROI ) )
            {
                hasCalib = true;
                // if ( overlays & CALIB_SCALE ||
                //      overlays & CALIB_GRID )
                // {
                //     string err_msg;
                //     retVal = m_visApp.DrawAssocPts( m_matColor, matTemp, err_msg );
                // }
                retVal = m_visApp.DrawCalibOverlay( matTemp, m_matDisplay,
                                                    overlays & CALIB_SCALE,
                                                    overlays & CALIB_GRID,
                                                    overlays & MOVE_ROIS,
                                                    overlays & SEARCH_ROI,
                                                    overlays & TARGET_ROI );
            }

            if ( hasCalib )
            {
                matTemp = m_matDisplay.clone();
            }
            int overlayType = OVERLAYS_NONE;
            if( overlays & FINDLINE )
                overlayType += FINDLINE;
            if( overlays & DIAG_ROWSUMS )
                overlayType += DIAG_ROWSUMS;
            if( overlays & FINDLINE_1ST_DERIV )
                overlayType += FINDLINE_1ST_DERIV;
            if( overlays & FINDLINE_2ND_DERIV )
                overlayType += FINDLINE_2ND_DERIV;
            if( overlays & RANSAC_POINTS )
                overlayType += RANSAC_POINTS;
            if( overlays & MOVE_FIND )
                overlayType += MOVE_FIND;
            if ( ( OVERLAYS_NONE != overlayType ) )
            {
                retVal = m_visApp.DrawLineFindOverlay( matTemp, m_matDisplay, static_cast< IMG_DISPLAY_OVERLAYS >( overlayType ) );
            }
            else
            {
                m_matDisplay = matTemp.clone();
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::GetImageColor( Mat matImgSrc, const Size sizeImg, const size_t nStride,
                                    const int nType, uchar *pPix, const bool bToRGB )
{
    GC_STATUS retVal = GC_OK;
    if ( nullptr == pPix )
    {
        FILE_LOG( logERROR ) << __func__ << "Cannot get an image to nullptr pixels";
        retVal = GC_ERR;
    }
    else if ( sizeImg != matImgSrc.size() )
    {
        FILE_LOG( logERROR ) << __func__ << "Invalid image dimension for GetImageColor()";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            if ( nType == CV_8UC3 )
            {
                uchar *pPixDst = pPix;
                size_t nBytes2Copy = static_cast< size_t >( sizeImg.width ) * 3;
                if ( bToRGB )
                {
                    Mat matRGB;
                    cvtColor( matImgSrc, matRGB, COLOR_BGR2RGB );
                    uchar *pPixSrc = matRGB.data;
                    for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                    {
                        memcpy( pPixDst, pPixSrc, static_cast< size_t >( nBytes2Copy ) );
                        pPixDst += nStride;
                        pPixSrc += static_cast< long >( matRGB.step );
                    }
                }
                else
                {
                    uchar *pPixSrc = matImgSrc.data;
                    for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                    {
                        memcpy( pPixDst, pPixSrc, nBytes2Copy );
                        pPixDst += nStride;
                        pPixSrc += static_cast< long >( matImgSrc.step );
                    }
                }
            }
            else if ( nType == CV_8UC4 )
            {
                int nCol4, nCol3;
                uchar *pPixDst = pPix;
                int nBytes2Copy = sizeImg.width * 3;
                if ( bToRGB )
                {
                    Mat matRGB;
                    cvtColor( matImgSrc, matRGB, COLOR_BGR2RGB );
                    uchar *pPixSrc = matRGB.data;
                    for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                    {
                        for ( nCol3 = 0, nCol4 = 0; nCol3 < nBytes2Copy; nCol3 += 3, nCol4 += 4 )
                        {
                            memcpy( &pPixDst[ nCol4 ], &pPixSrc[ nCol3 ], 3 );
                            pPixDst[ nCol4 + 3 ] = 0;
                        }
                        pPixDst += nStride;
                        pPixSrc += static_cast< long >( matRGB.step );
                    }
                }
                else
                {
                    uchar *pPixSrc = matImgSrc.data;
                    for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                    {
                        for ( nCol3 = 0, nCol4 = 0; nCol3 < nBytes2Copy; nCol3 += 3, nCol4 += 4 )
                        {
                            memcpy( &pPixDst[ nCol4 ], &pPixSrc[ nCol3 ], 3 );
                            pPixDst[ nCol4 + 3 ] = 0;
                        }
                        pPixDst += nStride;
                        pPixSrc += static_cast< long >( matImgSrc.step );
                    }
                }
            }
            else
            {
                FILE_LOG( logERROR ) << __func__ << "Invalid image type " << nType << " for GetImageColor()";
                retVal = GC_ERR;
            }
        }
        catch( const Exception &e )
        {
            FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS GuiVisApp::GetImageGray( Mat matImgSrc, const Size sizeImg, const int nStride, const int nType, uchar *pPix )
{
    GC_STATUS retVal = GC_OK;
    if ( nullptr == pPix )
    {
        FILE_LOG( logERROR ) << __func__ << "Cannot get an image to nullptr pixels";
        retVal = GC_ERR;
    }
    else if ( sizeImg != matImgSrc.size() || 0 > nStride )
    {
        FILE_LOG( logERROR ) << __func__ << "Invalid image dimension for GetImageGray()";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            if ( nType == CV_8UC3 )
            {
                int nCol, nCol3;
                uchar *pPixDst = pPix;
                uchar *pPixSrc = matImgSrc.data;
                for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                {
                    for ( nCol = 0, nCol3 = 0; nCol < sizeImg.width; ++nCol, nCol3 += 3 )
                        memset( &pPixDst[ nCol3 ], pPixSrc[ nCol ], 3 );
                    pPixDst += nStride;
                    pPixSrc += static_cast< long >( matImgSrc.step );
                }
            }
            else if ( nType == CV_8UC4 )
            {
                int nCol, nCol4;
                uchar *pPixDst = pPix;
                uchar *pPixSrc = matImgSrc.data;
                for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                {
                    for ( nCol = 0, nCol4 = 0; nCol < sizeImg.width; ++nCol, nCol4 += 4 )
                    {
                        memset( &pPixDst[ nCol4 ], pPixSrc[ nCol ], 3 );
                        pPixDst[ nCol4 + 3 ] = 0;
                    }
                    pPixDst += nStride;
                    pPixSrc += static_cast< long >( matImgSrc.step );
                }
            }
            else
            {
                FILE_LOG( logERROR ) << __func__ << "Invalid image type " << nType << " for SetImage()";
                retVal = GC_ERR;
            }
        }
        catch( const Exception &e )
        {
            FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS GuiVisApp::GetImageSize( Size &sizeImage )
{
    GC_STATUS retVal = GC_OK;
    if ( !IsInitialized() )
    {
        FILE_LOG( logERROR ) << __func__ << "Vision app must be initialized to retrieve image size";
        retVal = GC_ERR;
    }
    sizeImage = m_matGray.size();
    return retVal;
}
GC_STATUS GuiVisApp::LoadImageToApp( const string strFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        string filename = fs::path( strFilepath ).filename().string();
        Mat matTemp = imread( strFilepath, IMREAD_UNCHANGED );

        if ( matTemp.empty() )
        {
            FILE_LOG( logERROR ) << __func__ << "Could not read image " << strFilepath;
            retVal = GC_ERR;
        }
        else
        {
            retVal = LoadImageToApp( matTemp );
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::LoadImageToApp( const Mat img )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        std::lock_guard< std::mutex > lock( mtx_img );
        if ( img.empty() )
        {
            FILE_LOG( logERROR ) << __func__ << "Cannot load empty image to application";
            retVal = GC_ERR;
        }
        else
        {
            Mat matTemp;
            retVal = AdjustImageSize( img, matTemp );
            if ( GC_OK == retVal )
            {
                if ( img.size() != m_matGray.size() )
                {
                    retVal = InitBuffers( img.size() );
                    if ( GC_OK == retVal )
                    {
                        retVal = GC_WARN;
                    }
                }
                if ( GC_OK == retVal || GC_WARN == retVal )
                {
                    if ( CV_8UC1 == img.type() )
                    {
                        img.copyTo( m_matGray );
                        cvtColor( img, m_matColor, COLOR_GRAY2BGR );
                    }
                    else if ( CV_8UC3 == img.type() )
                    {
                        img.copyTo( m_matColor );
                        cvtColor( m_matColor, m_matGray, COLOR_BGR2GRAY );
                    }
                    else if ( CV_8UC4 == img.type() )
                    {
                        cvtColor( img, m_matColor, COLOR_BGRA2BGR );
                        cvtColor( m_matColor, m_matGray, COLOR_BGRA2GRAY );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << __func__ << "Invalid image type for LoadImage()";
                        retVal = GC_ERR;
                    }
                }
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::SaveImage( const string strFilepath, IMG_BUFFERS nColorType )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        bool bRet = false;
        switch( nColorType )
        {
            case BUF_GRAY: bRet = imwrite( strFilepath, m_matGray ); break;
            case BUF_RGB: bRet = imwrite( strFilepath, m_matColor ); break;
            case BUF_OVERLAY: bRet = imwrite( strFilepath, m_matDisplay ); break;
            default: break;
        }
        if ( !bRet )
        {
            FILE_LOG( logERROR ) << __func__ << "Could not save image " << strFilepath;
            retVal = GC_ERR;
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << __func__ << "EXCEPTION: " << string( e.what() );
        retVal = GC_EXCEPT;
    }
    return retVal;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Application settings
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
GC_STATUS GuiVisApp::ReadSettings( const string strJsonConfig )
{
    GC_STATUS retVal = GC_OK;
    FILE_LOG( logINFO ) << "Reading device config file from " << strJsonConfig;
    if ( strJsonConfig.empty() )
    {
        FILE_LOG( logINFO ) << __func__ << "Reading application settings from default file";
    }

    return retVal;
}
GC_STATUS GuiVisApp::WriteSettings( const string strJsonConfig )
{
    GC_STATUS retVal = GC_OK;
    FILE_LOG( logINFO ) << "Writing device config file to " << strJsonConfig;
    if ( strJsonConfig.empty() )
    {
        FILE_LOG( logINFO ) << __func__ << "Writing application settings to default file";
    }

    return retVal;
}
GC_STATUS GuiVisApp::AdjustImageSize( const Mat &matSrc, Mat &matDst )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( MAX_IMAGE_SIZE.width < matSrc.cols || MAX_IMAGE_SIZE.height < matSrc.rows )
        {
            double wideRatio = static_cast< double >( MAX_IMAGE_SIZE.width ) / static_cast< double >( matSrc.cols );
            double highRatio = static_cast< double >( MAX_IMAGE_SIZE.height ) / static_cast< double >( matSrc.rows );
            double imageRatio = ( wideRatio > highRatio ) ? wideRatio : highRatio;
            int newWide = cvRound( static_cast< double >( matSrc.cols ) * imageRatio );
            int newHigh = cvRound( static_cast< double >( matSrc.rows ) * imageRatio );
            resize( matSrc, matDst, Size( newWide, newHigh ) );
        }
        else
        {
            if ( &matDst != &matSrc )
                matDst = matSrc.clone();
        }
    }
    catch( Exception &e )
    {
        FILE_LOG( logERROR ) << " " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Application area -- Findline
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
GC_STATUS GuiVisApp::GetMetadata( const std::string imgFilepath, std::string &data )
{
    stringstream ss;
    ss << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    ss << "exif image features" << endl;
    ss << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    ExifFeatures exifFeats;
    GC_STATUS retVal = m_visApp.GetImageData( imgFilepath, exifFeats );
    ss << "Capture time: " << exifFeats.captureTime << endl;
    ss << "Exposure time: " << exifFeats.exposureTime << endl;
    ss << "fNumber: " << exifFeats.fNumber << endl;
    ss << "ISO speed rating: " << exifFeats.isoSpeedRating << endl;
    ss << "Image width: " << exifFeats.imageDims.width << endl;
    ss << "Image height: " << exifFeats.imageDims.height << endl;
    ss << "Shutter speed: " << exifFeats.shutterSpeed << endl;

    data = ss.str();
    sigMessage( string( "Metadata retrieval: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
GC_STATUS GuiVisApp::CreateAnimation( const std::string imageFolder, const std::string animationFilepath, const int delay_ms, const double scale )
{
    GC_STATUS retVal = GC_OK;
    if ( m_isRunning )
    {
        sigMessage( "Tried to run thread when it is already running" );
        FILE_LOG( logWARNING ) << "[GuiVisApp::CreateAnimation] Tried to run thread when it is already running";
        retVal = GC_WARN;
    }
    else
    {
        try
        {
            string ext;
            vector< std::string > images;
            for ( auto& p: fs::directory_iterator( imageFolder ) )
            {
                ext = p.path().extension().string();
                std::transform( ext.begin(), ext.end(), ext.begin(),
                                   []( unsigned char c ){ return std::tolower( c ); } );
                if ( ext == ".png" || ext == ".jpg" ||
                     ext == ".PNG" || ext == ".JPG" )
                {
                    images.push_back( p.path().string() );
                }
            }
            if ( images.empty() )
            {
                sigMessage( "No images found in specified folder" );
                FILE_LOG( logERROR ) << "[VGuiVisApp::CreateAnimation] No images found in specified folder";
                retVal = GC_ERR;
            }
            else
            {
                sort( images.begin(), images.end() );

                m_isRunning = true;
                m_folderFuture = std::async( std::launch::async, &GuiVisApp::CreateGIFThreadFunc, this, animationFilepath, images, delay_ms, scale );
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLinesFolder] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    //    GC_STATUS retVal = m_visApp.CreateAnimation( imageFolder, animationFilepath, delay_ms, scale );
    //    sigMessage( string( "Create animation: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
GC_STATUS GuiVisApp::GetTargetSearchROI( cv::Rect &rect )
{
    GC_STATUS retVal = m_visApp.GetTargetSearchROI( rect );
    if ( GC_OK == retVal )
    {
        if ( 0 > rect.x || m_matGray.cols < rect.width ||
             0 > rect.y || m_matGray.rows < rect.height )
        {
            sigMessage( string( "Invalid calibration search ROI" ) );
            retVal = GC_ERR;
        }
    }
    return retVal;
}

GC_STATUS GuiVisApp::GetCalibParams( std::string &calibParams )
{
    GC_STATUS retVal = m_visApp.GetCalibParams( calibParams );
    sigMessage( string( "Get calibration parameters: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
bool GuiVisApp::IsBowtieCalib() { return m_visApp.GetCalibType() == "BowTie"; }
GC_STATUS GuiVisApp::LoadCalib( const std::string calibJson, const bool reCalib )
{
    Mat noImg = Mat();
    GC_STATUS retVal = m_visApp.LoadCalib( calibJson, reCalib ? m_matColor : noImg );
    sigMessage( string( "Load calibration: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
GC_STATUS GuiVisApp::Calibrate( const std::string imgFilepath, const string jsonControl )
{
    GC_STATUS retVal = GC_OK;

    string err_msg;
    double rmseDist, rmseX, rmseY;
    retVal = LoadImageToApp( imgFilepath );
    if ( GC_OK == retVal )
    {
        retVal = m_visApp.Calibrate( imgFilepath, jsonControl, rmseDist, rmseX, rmseY, err_msg );
    }
    if ( GC_OK == retVal )
    {
        char msg[ 256 ];
        sprintf( msg, "X=%0.3e\nY=%0.3e\nEuclid. dist=%0.3e", rmseX, rmseY, rmseDist );
        sigMessage( string( "Calibration: SUCCESS\n" ) +
                    string( "~~~~~~~~~~~~~~~~~\n" ) +
                    string( "Reprojection RMSE\n" +
                    string( "~~~~~~~~~~~~~~~~~\n" ) +
                    string( msg ) +
                    string( "\n~~~~~~~~~~~~~~~~~\n" ) ) );
    }
    else
    {
        if ( err_msg.empty() )
            err_msg = "CALIB FAIL: Unknown error";
        sigMessage( err_msg );
    }
    return retVal;
}
GC_STATUS GuiVisApp::PixelToWorld( const cv::Point2d pixelPt, cv::Point2d &worldPt )
{
    GC_STATUS retVal = m_visApp.PixelToWorld( pixelPt, worldPt );
    return retVal;
}
GC_STATUS GuiVisApp::CalcLine( const FindLineParams params, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;

    retVal = m_visApp.CalcLine( params, result );
    if ( GC_OK == retVal )
    {
        GC_STATUS retVal1 = m_visApp.DrawLineFindOverlay( m_matColor, m_matDisplay );
        if ( GC_OK != retVal1 )
        {
            m_matDisplay = m_matColor.clone();
            putText( m_matDisplay, "Calc line OK, could not display result", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 1.8, Scalar( 0, 0, 255 ), 5 );
        }
        sigMessage( "Calculate level: SUCCESS" );
    }
    else
    {
        m_matDisplay = m_matColor.clone();
        putText( m_matDisplay, "Calc line FAILED", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 1.8, Scalar( 0, 0, 255 ), 5 );
        sigMessage( "Calculate level: FAILURE" );
    }
    return retVal;
}
GC_STATUS GuiVisApp::CalcLinesInFolder( const std::string folder, const FindLineParams params, const bool isFolderOfImages, const IMG_DISPLAY_OVERLAYS drawTypes )
{
    GC_STATUS retVal = GC_OK;
    if ( m_isRunning )
    {
        sigMessage( "Tried to run thread when it is already running" );
        FILE_LOG( logWARNING ) << "[GuiVisApp::CalcLinesInFolder] Tried to run thread when it is already running";
        retVal = GC_WARN;
    }
    else
    {
        try
        {
            string ext;
            vector< std::string > images;
            if ( isFolderOfImages )
            {
                for ( auto& p: fs::directory_iterator( folder ) )
                {
                    ext = p.path().extension().string();
                    std::transform( ext.begin(), ext.end(), ext.begin(),
                                       []( unsigned char c ){ return std::tolower( c ); } );
                    if ( ext == ".png" || ext == ".jpg" )
                    {
                        images.push_back( p.path().string() );
                    }
                }
            }
            else
            {
                for ( auto& f: fs::recursive_directory_iterator( folder ) )
                {
                    if ( fs::is_directory( f ) )
                    {
                        for ( auto& p: fs::recursive_directory_iterator( f ) )
                        {
                            ext = p.path().extension().string();
                            std::transform( ext.begin(), ext.end(), ext.begin(),
                                               []( unsigned char c ){ return std::tolower( c ); } );
                            if ( ext == ".png" || ext == ".jpg" )
                            {
                                images.push_back( p.path().string() );
                            }
                        }
                    }
                }
            }
            if ( images.empty() )
            {
                sigMessage( "No images found in specified folder" );
                FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesInFolder] No images found in specified folder";
                retVal = GC_ERR;
            }
            else
            {
                sort( images.begin(), images.end() );

                m_isRunning = true;
                m_folderFuture = std::async( std::launch::async, &GuiVisApp::CalcLinesThreadFunc, this, images, params, drawTypes );
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesInFolder] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS GuiVisApp::CreateGIFThreadFinish()
{
    GC_STATUS retVal = GC_OK;

    if ( !m_isRunning || ( m_isRunning && ( CREATE_GIF_THREAD != m_threadType ) ) )
    {
        sigMessage( "Tried to stop thread when it was not running" );
        FILE_LOG( logWARNING ) << "[VisApp::CreateGIFThreadFinish] Tried to stop thread when it was not running";
        retVal = GC_WARN;
    }
    else
    {
        try
        {
            m_isRunning = false;
            m_folderFuture.wait();
            retVal = m_folderFuture.get();
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CreateGIFThreadFinish] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS GuiVisApp::CalcLinesThreadFinish()
{
    GC_STATUS retVal = GC_OK;

    if ( !m_isRunning || ( m_isRunning && ( FIND_LINES_THREAD != m_threadType ) ) )
    {
        sigMessage( "Tried to stop thread when it was not running" );
        FILE_LOG( logWARNING ) << "[VisApp::CalcLinesThreadFinish] Tried to stop thread when it was not running";
        retVal = GC_WARN;
    }
    else
    {
        try
        {
            m_isRunning = false;
            m_folderFuture.wait();
            retVal = m_folderFuture.get();
            if ( GC_OK != retVal )
            {
                FILE_LOG( logERROR ) << "[VisApp::CalcLinesThreadFinish] Error in thread before termination";
                retVal = GC_OK;
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLinesThreadFinish] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
bool GuiVisApp::isRunningCreateGIF()
{
    return m_threadType != CREATE_GIF_THREAD ? false : m_isRunning;
}
GC_STATUS GuiVisApp::CreateGIFThreadFunc( const string gifFilepath, const std::vector< std::string > &images,  const int delay_ms, const double scale )
{
    GC_STATUS retVal = GC_OK;
    m_threadType = CREATE_GIF_THREAD;

    try
    {
        double progressVal = 0.0;
        sigProgress( cvRound( progressVal ) );

        if ( images.empty() )
        {
            m_isRunning = false;
            sigMessage( "No images in vector" );
            FILE_LOG( logERROR ) << "[VisApp::CreateGIFThreadFunc] No images in vector";
            retVal = GC_ERR;
        }
        else
        {
            Mat img = imread( images[ 0 ], IMREAD_COLOR );
            if ( img.empty() )
            {
                sigMessage( "Could not read first image " + images[ 0 ] );
                FILE_LOG( logERROR ) << "[VisApp::CreateGIFThreadFunc] Could not read first image " << images[ 0 ];
                retVal = GC_ERR;
            }
            else
            {
                resize( img, img, Size(), scale, scale, INTER_CUBIC );
                retVal = m_visApp.BeginGIF( img.size(), static_cast< int >( images.size() ), gifFilepath, delay_ms );
                if ( GC_OK == retVal )
                {
                    retVal = m_visApp.AddImageToGIF( img );
                    if ( GC_OK == retVal )
                    {
                        bool stopped = false;
                        for ( size_t i = 1; i < images.size(); ++i )
                        {
                            if ( !m_isRunning )
                            {
                                stopped = true;
                                break;
                            }
                            else
                            {
                                img = imread( images[ i ], IMREAD_COLOR );
                                if ( img.empty() )
                                {
                                    sigMessage( "Could not read image " + images[ i ] );
                                    FILE_LOG( logWARNING ) << "[VisApp::CreateGIFThreadFunc] Could not read image " << images[ i ];
                                }
                                else
                                {
                                    resize( img, img, Size(), scale, scale, INTER_CUBIC );
                                    retVal = m_visApp.AddImageToGIF( img );
                                    if ( GC_OK != retVal )
                                    {
                                        sigMessage( "Could not add image " + images[ i ] );
                                        FILE_LOG( logWARNING ) << "[VisApp::CreateGIFThreadFunc] Could not add image " << images[ i ];
                                    }
                                    else
                                    {
                                        sigMessage( "Added " + images[ i ] );
                                    }
                                    progressVal = 100.0 * static_cast< double >( i ) / static_cast< double >( images.size() ) + 1;
                                    sigProgress( cvRound( progressVal ) );
                                }
                            }
                        }
                        if ( !stopped )
                        {
                            sigMessage( "Create GIF complete" );
                            sigProgress( 100 );
                            m_isRunning = false;
                        }
                        else
                        {
                            sigMessage( "GIF stopped at " + to_string( progressVal ) + "%" );
                        }
                    }
                    retVal = m_visApp.EndGIF();
                    if ( GC_OK != retVal )
                    {
                        sigMessage( "End create GIF: FAIL" );
                    }
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CreateGIFThreadFunc] " << e.what();
        retVal = GC_EXCEPT;
    }
    m_threadType = NONE_RUNNING;

    return retVal;
}
bool GuiVisApp::isRunningFindLine()
{
    return m_threadType != FIND_LINES_THREAD ? false : m_isRunning;
}
inline GC_STATUS GuiVisApp::AccumRunImageCLIString( const FindLineParams params, std::string &cliString, std::string &resultFolder )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        cliString.clear();
        resultFolder.clear();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // example cli parameters
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // --find_line
        // --timestamp_from_exif
        // --timestamp_start_pos 0
        // --timestamp_format "yyyy-mm-dd-HH-MM"
        // --calib_json "./config/calib_stopsign.json"
        // --csv_file "/var/tmp/gaugecam/folder_stopsign.csv"
        // --source "./config/2022_demo/20220715_KOLA_GaugeCam_001.JPG"
        // --result_image "/var/tmp/gaugecam/find_line_result_stopsign.png"
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        cliString = "--find_line ";
        if ( FROM_FILENAME == params.timeStampType )
        {
            cliString += "--timestamp_from_filename ";
        }
        else if ( FROM_EXIF == params.timeStampType )
        {
            cliString += "--timestamp_from_exif ";
        }
        cliString += "--timestamp_start_pos ";
        cliString += to_string( params.timeStampStartPos );
        cliString += " --timestamp_format ";
        cliString += params.timeStampFormat;
        cliString += " --calib_json ";
        cliString += params.calibFilepath;
        if ( !params.resultCSVPath.empty() )
        {
            cliString += " --csv_file ";
            cliString += params.resultCSVPath;
        }
        cliString += " ";
        if ( !params.resultImagePath.empty() )
        {
            resultFolder = params.resultImagePath;
            if ( '/' != resultFolder[ resultFolder.size() - 1 ] )
            {
                resultFolder += '/';
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::AccumRunImageCLIString] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS GuiVisApp::CalcLinesThreadFunc( const std::vector< std::string > &images,  const FindLineParams params, const IMG_DISPLAY_OVERLAYS drawTypes )
{
    GC_STATUS retVal = GC_OK;
    m_threadType = FIND_LINES_THREAD;

    try
    {
        Mat img;
        string msg;
        int progressVal = 0;
        char buffer[ 256 ];
        bool stopped = false;

        // sort( images.begin(), images.end() );

        ofstream csvOut;

        if ( !params.resultCSVPath.empty() )
        {
            // cout << fs::path( params.resultCSVPath ).parent_path() << endl;
            if ( !fs::exists( fs::path( params.resultCSVPath ).parent_path() ) )
            {
                fs::create_directories( fs::path( params.resultCSVPath ).parent_path() );
            }
            csvOut.open( params.resultCSVPath );
            if ( !csvOut.is_open() )
            {
                FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesThreadFunc] Could not create CSV output file " << params.resultCSVPath;
                retVal = GC_ERR;
            }
            else
            {
                csvOut << "filename, timestamp, status, water level, line angle, level adjustment, illumination" << endl;
            }
        }
        if ( !params.resultImagePath.empty() )
        {
            if ( !fs::exists( params.resultImagePath ) )
            {
                bool bRet = fs::create_directories( params.resultImagePath );
                if ( !bRet )
                {
                    FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesThreadFunc] Could not create result folder " << params.resultImagePath;
                    retVal = GC_ERR;
                }
            }
            else if ( !fs::is_directory( params.resultImagePath ) )
            {
                FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesThreadFunc] Result path is not a folder " << params.resultImagePath;
                retVal = GC_ERR;
            }
        }

        if ( GC_OK == retVal )
        {
            if ( images.empty() )
            {
                sigMessage( "No images found" );
                retVal = GC_ERR;
            }
            else
            {
                img = imread( images[ 0 ], IMREAD_COLOR );
                if ( img.empty() )
                {
                    sigMessage( fs::path( images[ 0 ] ).filename().string() + " FAILURE: Could not open image to load calibration" );
                }
                else
                {
                    retVal = m_visApp.LoadCalib( params.calibFilepath, img );
                    if ( GC_OK != retVal )
                    {
                        sigMessage( "Failed to load calib for find line folder run" );
                    }
                    else
                    {
                        string cmdStringPrefix, cmdString, resultFolder;
                        retVal = AccumRunImageCLIString( params, cmdStringPrefix, resultFolder );
                        if ( GC_OK != retVal )
                        {
                            sigMessage( "Could not accumulate command line prefix string" );
                        }
                        for ( size_t i = 0; i < images.size(); ++i )
                        {
                            if ( !m_isRunning )
                            {
                                sigMessage( "Folder run stopped" );
                                stopped = true;
                                break;
                            }
                            else
                            {
                                cmdString = cmdStringPrefix + " --source ";
                                cmdString += images[ i ];
                                if ( !resultFolder.empty() )
                                {
                                    cmdString += " --result_image ";
                                    string filename = fs::path( images[ i ] ).stem().string() + "_overlay.png";
                                    fs::path full_path = fs::path( resultFolder ) / filename;
                                    cmdString += full_path.string();
                                    cmdString = "/media/kchapman/Elements/Projects/GRIME2/build-grime2cli-Desktop-Debug/grime2cli " + cmdString;
                                    int status = std::system( cmdString.c_str() );
                                    if (status < 0)
                                        std::cout << "Error: " << strerror(errno) << '\n';
                                    else
                                    {
                                        if (WIFEXITED(status))
                                            std::cout << "Program returned normally, exit code " << WEXITSTATUS(status) << '\n';
                                        else
                                            std::cout << "Program exited abnormaly\n";
                                    }
                                }
                            }
                            progressVal = cvRound( 100.0 * static_cast< double >( i ) / static_cast< double >( images.size() ) ) + 1;
                            sigProgress( progressVal );
                        }
                        if ( !stopped )
                        {
                            sigMessage( "Folder run complete" );
                            sigProgress( 100 );
                            m_isRunning = false;
                        }
                    }
                }
            }
        }
        if ( csvOut.is_open() )
            csvOut.close();
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLinesThreadFunc] " << e.what();
        retVal = GC_EXCEPT;
    }
    m_threadType = NONE_RUNNING;

    return retVal;
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Utility methods
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
GC_STATUS GuiVisApp::RemoveAllFilesInFolder( const string folderpath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        fs::recursive_directory_iterator rdi( folderpath );
        fs::recursive_directory_iterator end_rdi;

        for ( ; rdi != rdi; ++rdi )
        {
            if( fs::is_regular_file( rdi->status() ) )
                fs::remove( rdi->path() );
        }
    }
    catch( const boost::exception& e )
    {
        FILE_LOG( logERROR ) << "[GuiVisApp::RemoveAllFilesInFolder] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS GuiVisApp::Test()
{
    GC_STATUS retVal = GC_OK;

    auto start = boost::chrono::steady_clock::now();


    auto end = boost::chrono::steady_clock::now();
    auto diff = end - start;
    FILE_LOG( logINFO ) << "Elapsed time = " << \
                           boost::chrono::duration_cast < boost::chrono::milliseconds >( diff ).count();
    return retVal;
}

} // namespace gc
