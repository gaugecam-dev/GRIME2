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

#ifndef FEATUREDATA_H
#define FEATUREDATA_H

#include <string>
#include <vector>
#include <limits>
#include <opencv2/core.hpp>

static const std::string FEATURE_CALC_VERSION = "0.0.0.1";

namespace gc
{

class PixelStats
{
public:
    PixelStats() :
        centroid( cv::Point2d( -1.0, -1.0 ) ),
        average( -1.0 ),
        sigma( -1.0 ),
        verticalGradient( -1.0 ),
        horizontalGradient( -1.0 )
    {}

    PixelStats( const double avg, const double stdev, const cv::Point2d centerMass,
                const double vertGradient, const double horzGradient ) :
        centroid( centerMass ),
        average( avg ),
        sigma( stdev ),
        verticalGradient( vertGradient ),
        horizontalGradient( horzGradient )
    {}

    void clear()
    {
        centroid = cv::Point2d( -1.0, -1.0 );
        average = -1.0;
        sigma = -1.0;
        verticalGradient = -9999999.0;
        horizontalGradient = -9999999.0;
    }

    cv::Point2d centroid;
    double average;
    double sigma;
    double verticalGradient;
    double horizontalGradient;
};

class EdgeStats
{
public:
    EdgeStats() :
        centroid( cv::Point2d( -1.0, -1.0 ) ),
        meanDir( -1.0 ),
        meanMag( -1.0 )
    {}

    EdgeStats( const double avgDir, const double avgMag, const cv::Point2d centerMass ) :
        centroid( centerMass ),
        meanDir( avgDir ),
        meanMag( avgMag )
    {}

    void clear()
    {
        centroid = cv::Point2d( -1.0, -1.0 );
        meanDir = -1.0;
        meanMag = -1.0;
    }

    cv::Point2d centroid;
    double meanDir;
    double meanMag;
};

class ExifFeatures
{
public:
    ExifFeatures() :
        imageDims( -1, -1 ),
        captureTime( "" ),
        exposureTime( -1.0 ),
        fNumber( -1.0 ),
        isoSpeedRating( -1 ),
        shutterSpeed( -1.0 ),
        illumination( "N/A" )
    {}

    void clear()
    {
        imageDims = cv::Size( -1, -1 );
        captureTime.clear();
        exposureTime = -1.0;
        fNumber = -1.0;
        isoSpeedRating = -1;
        shutterSpeed = -1.0;
        illumination = "N/A";
    }
    cv::Size imageDims;
    std::string captureTime;
    double exposureTime;
    double fNumber;
    int isoSpeedRating;
    double shutterSpeed;
    std::string illumination;
};

class ImageAreaFeatures
{
public:
    ImageAreaFeatures() { clear(); }

    void clear()
    {
        name.clear();
        imageSize = cv::Size( -1, -1 );
        grayStats.clear();
        entropyStats.clear();
        hsvStats.clear();
        maskContour.clear();
    }

    std::string name;
    cv::Size imageSize;
    PixelStats grayStats;
    PixelStats entropyStats;
    std::vector< PixelStats > hsvStats;
    std::vector< cv::Point > maskContour;
};

class SensorDataItem
{
public:
    SensorDataItem() { clear(); }

    void clear()
    {
        timeStamp = "0000-00-00T00:00:00";
        value = -1.0;
    }
    std::string timeStamp;
    double value;
};

class SensorDataSet
{
public:
    SensorDataSet() { clear(); }

    void clear()
    {
        name.clear();
        items.clear();
    }
    std::string name;
    std::vector< SensorDataItem > items;
};

class Stats
{
public:
    Stats() { clear(); }
    Stats( const double minVal,
           const double maxVal,
           const double meanVal,
           const double sigmaVal ) :
        min( minVal ),
        max( maxVal ),
        mean( meanVal ),
        sigma( sigmaVal )
    {}


    void clear()
    {
        min = -std::numeric_limits< double >::max();
        max = -std::numeric_limits< double >::max();
        mean = -std::numeric_limits< double >::max();
        sigma = -std::numeric_limits< double >::max();
    }

    double min;
    double max;
    double mean;
    double sigma;
};

class FeatureSet
{
public:
    FeatureSet() : featureCalcVersion( FEATURE_CALC_VERSION ) {}

    void clear()
    {
        imageFilename.clear();
        imgTimestamp.clear();
        calcTimestamp.clear();
        imageSize = cv::Size( -1, -1 );
        exif.clear();
        sensorData.clear();
        areaFeats.clear();
    }

    std::string imageFilename;
    std::string imgTimestamp;
    std::string calcTimestamp;
    cv::Size imageSize;
    std::string featureCalcVersion;

    ExifFeatures exif;
    std::vector< SensorDataSet > sensorData;
    std::vector< ImageAreaFeatures > areaFeats;
};

} // namespace gc

#endif // FEATUREDATA_H
