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

#ifndef GUIVISAPP_H
#define GUIVISAPP_H

#include <thread>
#include <vector>
#include <future>
#include <condition_variable>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <boost/signals2.hpp>
#include "../algorithms/visapp.h"
#include "../algorithms/visappfeats.h"
#include "../algorithms/labelroi.h"

static const cv::Size MAX_IMAGE_SIZE = cv::Size( 1280, 1280 );

namespace gc
{

static const std::string GAUGECAM_GUI_VISAPP_VERSION = "0.0.0.1";

enum IMG_BUFFERS
{
    BUF_GRAY = 0,
    BUF_RGB,
    BUF_OVERLAY,
    BUF_DISPLAY
};
enum IMG_DISPLAY_OVERLAYS
{
    OVERLAYS_NONE = 0,
    CALIB = 1,
    FINDLINE = 2,
    FEATROIS = 4,
    MOVEMENT = 8
};

class GuiVisApp
{
public:
    typedef boost::signals2::signal< void( const std::string ) > VisAppMsgSig;
    typedef boost::signals2::signal< void( const int ) > VisAppProgressSig;
    typedef boost::signals2::signal< void( const string ) > VisAppUpdateTable;

    GuiVisApp();
    ~GuiVisApp();
    std::string Version() { return GAUGECAM_GUI_VISAPP_VERSION; }

    GC_STATUS Init( const std::string strConfigFolder, cv::Size &sizeImg );
    GC_STATUS Destroy();
    bool IsInitialized();
    GC_STATUS ReadSettings( const std::string strJsonConfig );
    GC_STATUS WriteSettings( const std::string strJsonConfig = "" );
    GC_STATUS Test();

    GC_STATUS SetImage( const cv::Mat matImg, const bool bIsBGR = true );
    GC_STATUS SetImage( const cv::Size sizeImg, const size_t nStride, const int nType, uchar *pPix, const bool bIsBGR = true );
    GC_STATUS GetImage( const cv::Size sizeImg, const size_t nStride, const int nType, uchar *pPix,
                        const IMG_BUFFERS nImgColor, const IMG_DISPLAY_OVERLAYS overlays, const int roiIndex = -1 );
    cv::Mat &GetImageFromType( IMG_BUFFERS type );

    GC_STATUS LoadImageToApp( const cv::Mat img );
    GC_STATUS LoadImageToApp( const std::string strFilepath );
    GC_STATUS SaveImage( const std::string strFilepath, IMG_BUFFERS nColorType );
    GC_STATUS GetImageSize( cv::Size &sizeImage );

    std::string ConfigFolder() { return m_strConfigFolder; }

    // findline app methods
    GC_STATUS GetMetadata( const std::string imgFilepath, std::string &data );
    GC_STATUS LoadCalib( const std::string calibJson );
    GC_STATUS Calibrate( const std::string imgFilepath, const std::string worldCoordsCsv, const std::string &calibJson );
    GC_STATUS CalcLine( const FindLineParams params, FindLineResult &result );
    GC_STATUS CalcLinesInFolder( const std::string folder, const FindLineParams params, const bool isFolderOfImages );
    GC_STATUS CalcLinesThreadFinish();
    bool isRunningFindLine();

    // kalman app methods
    GC_STATUS ApplyKalman( const std::string inputCSV, const std::string outputCSV, const std::string timestampFormat,
                           const int inputTimestampCol, const int inputMeasureCol );

    // recipe creation and feature calculation
    void ClearROIFeatList();
    GC_STATUS ReadCalcFeatures( const std::string filepath );
    GC_STATUS WriteCalcFeatures( const std::string filepath );
    cv::Rect GetAnchorROI();
    GC_STATUS SetAnchorRef( const std::string imgFilepath, const cv::Rect roi );
    GC_STATUS LoadROIFeatFromJson( const std::string filepath, std::vector< LabelROIItem > &items );
    GC_STATUS CalcFeaturesInFolder( const std::string folder, const FeatCalcParams params, const bool isFolderOfImages );

    // signals
    VisAppMsgSig sigMessage;
    VisAppProgressSig sigProgress;
    VisAppUpdateTable sigTableAddRow;

private:
    VisApp m_visApp;
    VisAppFeats m_visAppFeats;
    cv::Mat m_matGray;
    cv::Mat m_matColor;
    cv::Mat m_matDisplay;

    bool m_isRunning;
    std::future< GC_STATUS > m_folderFuture;

    bool m_bShowRuler;

    std::vector< std::string > m_vecImgPaths;
    std::vector< LabelROIItem > m_labeledROIitems;

    std::string m_strConfigFolder;
    std::string m_strCurrentImageFilepath;
    std::string m_instanceCacheFolder;

    FILE *m_pFileLog;

    // helper methods
    GC_STATUS GetImageColor( cv::Mat matImgSrc, const cv::Size sizeImg, const size_t nStride,
                       const int nType, uchar *pPix, const bool bToRGB );
    GC_STATUS GetImageGray( cv::Mat matImgSrc, const cv::Size sizeImg, const int nStride,
                      const int nType, uchar *pPix );
    GC_STATUS GetImageOverlay( const IMG_BUFFERS nImgColor, const IMG_DISPLAY_OVERLAYS overlays = OVERLAYS_NONE, const int roiIndex = -1 );
    GC_STATUS InitBuffers( const cv::Size sizeImg );
    GC_STATUS AdjustImageSize( const cv::Mat &matSrc, cv::Mat &matDst );
    GC_STATUS RemoveAllFilesInFolder( const std::string folderpath );
    GC_STATUS CalcLinesThreadFunc( const std::vector< std::string > &images, const FindLineParams params );
};

} // namespace gc

#endif // GUIVISAPP_H
