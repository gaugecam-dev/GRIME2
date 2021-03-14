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
#include <string>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "../algorithms/log.h"
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include "../algorithms/timestampconvert.h"
#include "../algorithms/kalman.h"

using namespace cv;
using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

#ifndef DEBUG_PYTHON_GRAPH_SERVER
#define DEBUG_PYTHON_GRAPH_SERVER
#endif
#ifdef _WIN32
static const string LOG_FILE_FOLDER = "c:/temp/";
#else
static const string LOG_FILE_FOLDER = "/var/tmp/";
#endif

using namespace cv;
using namespace std;

std::mutex mtx_img;

namespace gc
{

GuiVisApp::GuiVisApp() :
    m_isRunning( false ),
    m_bShowRuler( false ),
    m_strConfigFolder( "./config" ),
    m_strCurrentImageFilepath( "" ),
    m_pFileLog( nullptr )
{
    bool graphServerOk = true;

    fs::path p( LOG_FILE_FOLDER );
    bool folderExists = fs::exists( p );
    if ( !folderExists )
        folderExists = fs::create_directories( p );

    char buf[ 256 ];
    sprintf( buf, "%svisapp.log", folderExists ? LOG_FILE_FOLDER.c_str() : "" );

    //    m_pFileLog = fopen( buf, "w" );
    //    Output2FILE::Stream() = m_pFileLog;
    Output2FILE::Stream() = stdout;

    if ( !graphServerOk )
    {
        FILE_LOG( logWARNING ) << "[GuiVisApp()] Could not start graph server";
    }
}
GuiVisApp::~GuiVisApp()
{
    Destroy();
}
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
GC_STATUS GuiVisApp::SetImage( const Mat matImg, const bool bIsBGR )
{
    GC_STATUS retVal = GC_OK;
    if ( matImg.size() != m_matGray.size() )
    {
        FILE_LOG( logERROR ) << "[SetImage] Invalid image size for image set";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            if ( matImg.type() == CV_8UC1 )
            {
                matImg.copyTo( m_matGray );
                cvtColor( matImg, m_matColor, COLOR_GRAY2BGR );
            }
            else if ( matImg.type() == CV_8UC3 )
            {
                matImg.copyTo( m_matColor );
                cvtColor( matImg, m_matGray, bIsBGR ? COLOR_BGR2GRAY : COLOR_RGB2GRAY );
            }
            else
            {
                FILE_LOG( logERROR ) << __func__ << "Invalid image type for SetImage()";
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
GC_STATUS GuiVisApp::SetImage( const Size sizeImg, const size_t nStride, const int nType, uchar *pPix, const bool bIsBGR )
{
    GC_STATUS retVal = GC_OK;
    if ( nullptr == pPix )
    {
        FILE_LOG( logERROR ) << __func__ << "Cannot set an image from nullptr pixels";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            if ( nType == CV_8UC1 )
            {
                uchar *pPixSrc = pPix;
                uchar *pPixDst = m_matGray.data;
                for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                {
                    memcpy( pPixDst, pPixSrc, static_cast< size_t >( sizeImg.width ) );
                    pPixSrc += nStride;
                    pPixDst += static_cast< long >( m_matGray.step );
                }
                cvtColor( m_matGray, m_matColor, COLOR_GRAY2BGR );
            }
            else if ( nType == CV_8UC3 )
            {
                uchar *pPixSrc = pPix;
                uchar *pPixDst = m_matColor.data;
                size_t nBytes2Copy = static_cast< size_t >( sizeImg.width ) * 3;
                for ( int nRow = 0; nRow < sizeImg.height; ++nRow )
                {
                    memcpy( pPixDst, pPixSrc, nBytes2Copy );
                    pPixSrc += nStride;
                    pPixDst += static_cast< long >( m_matColor.step );
                }
                cvtColor( m_matColor, m_matGray, bIsBGR ? COLOR_BGR2GRAY : COLOR_RGB2GRAY );
            }
            else
            {
                FILE_LOG( logERROR ) << string( "Invalid image type " ) << nType << string( " for SetImage()" );
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
cv::Mat &GuiVisApp::GetImageFromType( IMG_BUFFERS type )
{
    Mat *pMatRet = nullptr;
    switch( type )
    {
        case BUF_GRAY:    pMatRet = &m_matGray; break;
        case BUF_RGB:     pMatRet = &m_matColor; break;
        case BUF_OVERLAY: pMatRet = &m_matDisplay; break;
        default: break;
    }
    return *pMatRet;
}
GC_STATUS GuiVisApp::GetImage(const Size sizeImg, const size_t nStride, const int nType,
                               uchar *pPix, const IMG_BUFFERS nImgColor, const IMG_DISPLAY_OVERLAYS overlays )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        retVal = GetImageOverlay( nImgColor, overlays );
        if ( 0 != retVal )
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
            m_matDisplay = m_matColor.clone();
            if ( overlays & CALIB )
            {
                retVal = m_visApp.DrawCalibOverlay( m_matDisplay, m_matDisplay );
            }
            if ( overlays & FINDLINE )
            {
                retVal = m_visApp.DrawLineFindOverlay( m_matDisplay, m_matDisplay );
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
    GC_STATUS retVal = m_visApp.GetExifImageData( imgFilepath, exifFeats );
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
GC_STATUS GuiVisApp::LoadCalib( const std::string calibJson )
{
    GC_STATUS retVal = m_visApp.LoadCalib( calibJson );
    sigMessage( string( "Load calibration: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
GC_STATUS GuiVisApp::Calibrate( const std::string imgFilepath, const std::string worldCoordsCsv, const std::string &calibJson )
{
    GC_STATUS retVal = GC_OK;

    Mat matOut;
    retVal = LoadImageToApp( imgFilepath );
    if ( GC_OK == retVal )
    {
        retVal = m_visApp.Calibrate( imgFilepath, worldCoordsCsv, calibJson, matOut );
    }
    sigMessage( string( "Calibration: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    return retVal;
}
GC_STATUS GuiVisApp::CalcLine( const FindLineParams params, FindLineResult &result )
{
    GC_STATUS retVal = GC_OK;

    string timestamp = "YYYY-MM-DDThh:mm:ss";
    if ( FROM_FILENAME == params.timeStampType )
    {
        retVal = GcTimestampConvert::GetTimestampFromString( fs::path( params.imagePath ).filename().string(),
                                                             params.timeStampStartPos, params.timeStampLength,
                                                             params.timeStampFormat, timestamp );
    }
    else if ( FROM_EXIF == params.timeStampType )
    {
        string timestampTemp;
        retVal = m_visApp.GetExifTimestamp( params.imagePath, timestampTemp );
        if ( GC_OK == retVal )
        {
            retVal = GcTimestampConvert::GetTimestampFromString( timestampTemp,
                                                                 params.timeStampStartPos, params.timeStampLength,
                                                                 params.timeStampFormat, timestamp );
        }
    }
    else if ( FROM_EXTERNAL == params.timeStampType )
    {
        FILE_LOG( logERROR ) << "Timestamp passed into method not yet implemented";
        retVal = GC_ERR;
    }
    if ( GC_OK != retVal )
    {
        FILE_LOG( logWARNING ) << "Unable to read timestamp";
    }
    retVal = m_visApp.CalcLine( params, timestamp, result );
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
GC_STATUS GuiVisApp::CalcLinesInFolder( const std::string folder, const FindLineParams params, const bool isFolderOfImages )
{
    GC_STATUS retVal = GC_OK;
    if ( m_isRunning )
    {
        sigMessage( "Tried to run thread when it is already running" );
        FILE_LOG( logWARNING ) << "[VisApp::CalcLinesThreadFinish] Tried to run thread when it is already running";
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
                for ( auto& p: fs::recursive_directory_iterator( folder ) )
                {
                    ext = p.path().extension().string();
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
                FILE_LOG( logERROR ) << "[VisApp::CalcLinesThreadFinish] No images found in specified folder";
                retVal = GC_ERR;
            }
            else
            {
                sort( images.begin(), images.end() );

                m_isRunning = true;
                m_folderFuture = std::async( std::launch::async, &GuiVisApp::CalcLinesThreadFunc, this, images, params );
            }
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLinesFolder] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS GuiVisApp::CalcLinesThreadFinish()
{
    GC_STATUS retVal = GC_OK;

    if ( !m_isRunning )
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
        }
        catch( std::exception &e )
        {
            FILE_LOG( logERROR ) << "[VisApp::CalcLinesThreadFinish] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
bool GuiVisApp::isRunningFindLine()
{
    return m_isRunning;
}
GC_STATUS GuiVisApp::CalcLinesThreadFunc( const std::vector< std::string > &images,  const FindLineParams params )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        retVal = m_visApp.LoadCalib( params.calibFilepath );
        if ( GC_OK != retVal )
        {
            sigMessage( "Failed to load calib for find line folder run" );
        }
        else if ( images.empty() )
        {
            sigMessage( "No images found" );
            retVal = GC_ERR;
        }
        else
        {
            Mat img;
            string msg;
            int progressVal = 0;
            char buffer[ 256 ];
            bool stopped = false;

            ofstream csvOut;
            string resultFolderAdj = params.resultImagePath;

            if ( !params.resultCSVPath.empty() )
            {
                csvOut.open( params.resultCSVPath );
                if ( !csvOut.is_open() )
                {
                    FILE_LOG( logERROR ) << "[GuiVisApp::CalcLinesThreadFunc] Could not create CSV output file " << params.resultCSVPath;
                    retVal = GC_ERR;
                }
                else
                {
                    csvOut << "filename, timestamp, seconds since epoch, water level, kalman estimate" << endl;
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
                if ( '/' != resultFolderAdj[ resultFolderAdj.size() - 1 ] )
                    resultFolderAdj += '/';

                KalmanItem item;
                KalmanFilter kf( 4, 2, 0 );
                kf.transitionMatrix = ( Mat_< float >( 4, 4 ) << 1,0,1,0,  0,1,0,1,  0,0,1,0,  0,0,0,1 );
                Mat_< float > measurement( 2, 1 );
                measurement.setTo( Scalar( 0 ) );

                FindData findData;
                findData.calibSettings = m_visApp.GetCalibModel();
                findData.findlineParams = params;

                GC_STATUS retTimeGet;
                string tmStr, timestamp, resultString, graphData;
                for ( size_t i = 0; i < images.size(); ++i )
                {
                    if ( !m_isRunning )
                    {
                        sigMessage( "Folder run stopped" );
                        i = images.size();
                        stopped = true;
                        break;
                    }
                    else
                    {
                        findData.findlineResult.clear();
                        string filename = fs::path( images[ i ] ).filename().string();
                        timestamp = "yyyy-mm-ddTHH:MM:SS";
                        if ( FROM_FILENAME == params.timeStampType )
                        {
                            retVal = GcTimestampConvert::GetTimestampFromString( fs::path( images[ i ] ).filename().string(),
                                                                                 params.timeStampStartPos, params.timeStampLength,
                                                                                 params.timeStampFormat, timestamp );
                        }
                        else if ( FROM_EXIF == params.timeStampType )
                        {
                            string timestampTemp;
                            retVal = m_visApp.GetExifTimestamp( params.imagePath, timestampTemp );
                            if ( GC_OK == retVal )
                            {
                                retVal = GcTimestampConvert::GetTimestampFromString( timestampTemp,
                                                                                     params.timeStampStartPos, params.timeStampLength,
                                                                                     params.timeStampFormat, timestamp );
                            }
                        }
                        img = imread( images[ i ], IMREAD_GRAYSCALE );
                        if ( img.empty() )
                        {
                            sigMessage( fs::path( images[ i ] ).filename().string() + " FAILURE: Could not open image" );
                        }
                        else
                        {
                            if ( FROM_EXTERNAL == params.timeStampType )
                            {
                                FILE_LOG( logERROR ) << "Timestamp passed into method not yet implemented";
                                retVal = GC_ERR;
                            }

                            sprintf( buffer, "Timestamp=%s\n", findData.findlineResult.timestamp.c_str() );
                            msg += string( buffer );

                            resultString = filename + ",";
                            resultString += timestamp + ",";

                            retVal = m_visApp.CalcLine( img, timestamp );
                            msg = filename + ( GC_OK == retVal ? " SUCCESS\n" : " FAILURE\n" );
                            if ( GC_OK == retVal )
                            {
                                findData.findlineResult = m_visApp.GetFindLineResult();

                                sprintf( buffer, "Water level=%.3f\n", findData.findlineResult.waterLevelAdjusted.y );
                                msg += string( buffer );
                                resultString += to_string( findData.findlineResult.waterLevelAdjusted.y );
                                sprintf( buffer, "Target movement x=%.3f, y=%.3f\n",
                                         findData.findlineResult.offsetMovePts.ctrWorld.x, findData.findlineResult.offsetMovePts.ctrWorld.y );
                                msg += string( buffer );
                                retTimeGet = GcTimestampConvert::ConvertDateToSeconds( fs::path( images[ i ] ).filename().string(),
                                                                                       params.timeStampStartPos, params.timeStampLength,
                                                                                       params.timeStampFormat, item.secsSinceEpoch );
                                if ( GC_OK == retTimeGet )
                                {
                                    sprintf( buffer, "Timestamp=%s\nSecs from epoch=%lld",
                                             findData.findlineResult.timestamp.c_str(), item.secsSinceEpoch );
                                }
                                else
                                {
                                    sprintf( buffer, "Timestamp=FAILED CONVERSION\nSecs from epoch=FAILED CONVERSION" );
                                }
                                msg += string( buffer );
                                item.measurement = findData.findlineResult.waterLevelAdjusted.y;
                            }
                            else
                            {
                                msg += string( "Water level=FAIL" );
                                resultString += to_string( -9999999.0 );
                            }

                            findData.findlineResult.timestamp = timestamp;
                            if ( !params.resultCSVPath.empty() )
                            {
                                csvOut << filename << "," << timestamp << "," << item.secsSinceEpoch << "," <<
                                          findData.findlineResult.waterLevelAdjusted.y << "," << item.prediction << endl;
                            }
                            if ( !params.resultImagePath.empty() )
                            {
                                Mat color;
                                retVal = m_visApp.DrawLineFindOverlay( img, color, findData.findlineResult );
                                if ( GC_OK == retVal )
                                {
                                    string resultFilepath = resultFolderAdj + fs::path( images[ i ] ).stem().string() + "_overlay.png";
                                    bool bRet = imwrite( resultFilepath, color );
                                    if ( !bRet )
                                    {
                                        FILE_LOG( logWARNING ) << "Could not write result image to " << resultFilepath;
                                    }
                                }
                            }

                            findData.findlineParams.imagePath = images[ i ];
                            retVal = LoadImageToApp( img );
                            sigMessage( "update image only" );
                            sigMessage( msg );
                            sigTableAddRow( resultString );
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
                if ( csvOut.is_open() )
                    csvOut.close();
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[VisApp::CalcLinesFolder] " << e.what();
        retVal = GC_EXCEPT;
    }

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
    FILE_LOG( logINFO ) << "Elapsed time for vertices extraction = " << \
                           boost::chrono::duration_cast < boost::chrono::milliseconds >( diff ).count();
    return retVal;
}

} // namespace gc
