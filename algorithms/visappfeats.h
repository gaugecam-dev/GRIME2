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

#ifndef VISAPPFEATS_H
#define VISAPPFEATS_H

#include "labelroi.h"
#include "featuredata.h"
#include "findanchor.h"

namespace gc
{

class FeatCalcItem
{
public:
    FeatCalcItem() :
        datetimeOriginal( std::string( "1955-09-24T12:05:00" ) ),
        datetimeProcessing( std::string( "1955-09-24T12:05:01" ) )
    {}

    FeatCalcItem( const std::string timeStampOriginal,
                  const std::string timeStampProcessing,
                  const std::string imageFilepath,
                  const std::string resultImageFilepath = "",
                  const std::string resultCSVFilepath = "" ) :
        datetimeOriginal( timeStampOriginal ),
        datetimeProcessing( timeStampProcessing ),
        imagePath( imageFilepath ),
        resultImagePath( resultImageFilepath ),
        resultCSVPath( resultCSVFilepath )
    {
    }

    void clear()
    {
        datetimeOriginal = std::string( "1955-09-24T12:05:00" );
        datetimeProcessing = std::string( "1955-09-24T12:05:01" );
        imagePath.clear();
        resultImagePath.clear();
        resultCSVPath.clear();
    }

    std::string datetimeOriginal;       ///< Datetime stamp when the image to be search was created
    std::string datetimeProcessing;     ///< Datetime stamp when the findline was performed
    std::string imagePath;              ///< Input image filepath of the image to be searched
    std::string resultImagePath;        ///< Optional result image created from input image with found line and move detection overlays
    std::string resultCSVPath;          ///< Optional result csv file path to hold timestamps and stage measurements
};

class FeatCalcParams
{
public:
    FeatCalcParams() :
        timeStampType( FROM_EXIF ),
        timeStampStartPos( -1 ),
        timeStampLength( -1 )
    {}

    // TODO -- Correct this documentation
    /**
     * @brief Constructor to set the object to a valid state
     * @param timeStampOriginal     Datetime stamp when the image to be search was created
     * @param timeStampProcessing   Datetime stamp when the findline was performed
     * @param imageFilepath         Input image filepath of the image to be searched
     * @param resultImageFilepath   Optional result image created from input image with found line and move detection overlays
     * @param resultCSVFilepath     Optional result CSV file to build a table of features for a set of images
     */
    FeatCalcParams( const GC_TIMESTAMP_TYPE tmStampType,
                    const int tmStampStartPos,
                    const int tmStampLength,
                    const std::string tmStampFormat,
                    const std::vector< LabelROIItem > rois ) :
        timeStampType( tmStampType ),
        timeStampStartPos( tmStampStartPos ),
        timeStampLength( tmStampLength ),
        timeStampFormat( tmStampFormat )
    {
        areaROIs.clear();
        for ( size_t i = 0; i < rois.size(); ++i )
            areaROIs.push_back( rois[ i ] );
    }

    void clear()
    {
        timeStampType = FROM_EXIF;
        timeStampStartPos = -1;
        timeStampLength = -1;
        timeStampFormat.clear();
        areaROIs.clear();
    }

    // timestamp in ISO 8601 DateTime format
    // e.g. 1955-09-24T12:05:00Z00:00
    GC_TIMESTAMP_TYPE timeStampType;    ///< Specifies where to get timestamp (filename, exif, or dateTimeOriginal)
    int timeStampStartPos;              ///< start position of timestamp string in filename (not whole path)
    int timeStampLength;                ///< length of timestamp string in filename
    std::string timeStampFormat;        ///< Format of the timestamp string, e.g. YYYY-MM-DDThh:mm::ss
    vector< LabelROIItem > areaROIs;    ///< Vector of regions of interest for area feature calculation
};

class VisAppFeats
{
public:
    VisAppFeats();

    GC_STATUS ReadSettings( const std::string jsonFilepath );
    GC_STATUS WriteSettings( const std::string jsonFilepath );
    GC_STATUS CalcMovement( const cv::Mat img, Point &ptOrig, cv::Point &ptMove, double &angle );
    cv::Rect GetAnchorROI() { return anchor.ModelRect(); }
    GC_STATUS SetAnchorRef( const string imgRefFilepath, const cv::Rect rect );
    GC_STATUS SetFeatROIs( const std::vector< LabelROIItem > &items );
    void SetCalcParams( const FeatCalcParams &params ) { featCalcParams = params; }
    GC_STATUS CreateCSVFileAndHeader( const std::string filepath, const FeatureSet &featSet );
    GC_STATUS WriteFeatSetToCSV( const std::string filepath, const FeatureSet &featSet );
    GC_STATUS ReadCSV( const std::string filepath, std::vector< FeatureSet > &featSets );

private:
    FindAnchor anchor;
    FeatCalcParams featCalcParams;

    GC_STATUS ParseRow( const std::vector< std::string > data, FeatureSet &feat );
};

} // namespace gc

#endif // VISAPPFEATS_H
