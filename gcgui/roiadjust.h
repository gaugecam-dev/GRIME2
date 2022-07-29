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

#ifndef ROIADJUST_H
#define ROIADJUST_H

#include <QSize>
#include <QLine>
#include <QRect>
#include <QPoint>
#include <string>
#include <opencv2/core.hpp>

typedef enum CALIB_JSON_STRING_TYPE
{
    BOWTIE,
    STOPSIGN
} CalibJsonStringType;

class LineSearchPoly
{
public:
    LineSearchPoly() :
        lftTop( QPoint( 50, 50 ) ),
        rgtTop( QPoint( 100, 50 ) ),
        rgtBot( QPoint( 100, 100 ) ),
        lftBot( QPoint( 50, 100 ) )
    {}

    LineSearchPoly( const QPoint lftTop, const QPoint rgtTop,
                    const QPoint rgtBot, const QPoint lftBot ) :
        lftTop( lftTop ), rgtTop( rgtTop ), rgtBot( rgtBot ), lftBot( lftBot )
    {}

    QPoint lftTop;
    QPoint rgtTop;
    QPoint rgtBot;
    QPoint lftBot;
};

class CalibJsonItems
{
public:
    CalibJsonItems() {}
    CalibJsonItems( const std::string worldPosCsvFile,
                    const std::string calibResultJson,
                    const bool enableROI,
                    const cv::Rect rect,
                    const double moveGrowROIPercent,
                    const double worldFacetLength,
                    const double worldZeroOffset,
                    const LineSearchPoly searchPoly,
                    const cv::Scalar trgtColor,
                    const int colorRngMin,
                    const int colorRngMax ) :
        worldTargetPosition_csvFile( worldPosCsvFile ),
        calibVisionResult_json( calibResultJson ),
        useROI( enableROI ),
        roi( rect ),
        moveROIGrowPercent( moveGrowROIPercent ),
        facetLength( worldFacetLength ),
        zeroOffset( worldZeroOffset ),
        lineSearchPoly( searchPoly ),
        targetColor( trgtColor ),
        colorRangeMin( colorRngMin ),
        colorRangeMax( colorRngMax )
    {}

    std::string worldTargetPosition_csvFile;
    std::string calibVisionResult_json;
    bool useROI;
    cv::Rect roi;
    double moveROIGrowPercent;
    double facetLength;
    double zeroOffset;
    LineSearchPoly lineSearchPoly;
    cv::Scalar targetColor;
    double colorRangeMin;
    double colorRangeMax;
};

class RoiAdjust
{
public:
    RoiAdjust();

    int FormBowtieCalibJsonString( const CalibJsonItems &items, std::string &json );
    int FormStopsignCalibJsonString( const CalibJsonItems &items, std::string &json );

    int TestAgainstFindLines( const QPoint pt, const QSize displaySize, const int capturePos,
                              const double scale, QPoint &ptCapture, QLine &lineOne );
    int TestAgainstRubberBands( QPoint pt, const QSize displaySize, QRect &rectRubberBand, QRect &rectROI,
                                const int capturePos, const double scale, QPoint &ptCapture );
    int TestAgainstPoly( QPoint pt, const QSize displaySize, LineSearchPoly &guiPoly,
                         const int capturePos, const double scale, QPoint &ptCapture );

    int EvalRectCapturePt( const QRect rectRubberBand, const QPoint ptAdj,
                           const int captureRadius, int &capturePos, QPoint &ptCapture );
    int EvalRulerCapturePt( const QLine lineOne, const QPoint ptAdj, const double scale,
                            const int captureRadius, int &capturePos, QPoint &ptCapture );
    int EvalPolyCapturePt( const LineSearchPoly guiPoly, const QPoint ptAdj, const double scale,
                           const int captureRadius, int &capturePos, QPoint &ptCapture );

private:
    int AdjustPointFindLines( const QSize displaySize, const double scale, QLine &lineOne );
    int AdjustPointRubberBand( const QSize displaySize, QRect &rectRubberBand );
    int AdjustPointPoly( const QSize displaySize, const double scale, LineSearchPoly &guiPoly );
};

#endif // ROIADJUST_H
