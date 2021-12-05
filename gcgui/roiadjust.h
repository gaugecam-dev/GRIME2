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

typedef enum CALIB_JSON_STRING_TYPE
{
    BOWTIE,
    STOPSIGN
} CalibJsonStringType;

class RoiAdjust
{
public:
    RoiAdjust();

    int FormBowtieCalibJsonString( const std::string csvFilepath, const std::string jsonResultFilepath,
                                   const bool useSearchROI, const QRect rectROI, std::string &json );
    int FormStopsignCalibJsonString( const std::string csvFilepath, const std::string jsonResultFilepath,
                                     const bool useSearchROI, const QRect rectROI,
                                     const bool fromFacetLength, const double facetLength, std::string &json );
    int TestAgainstFindLines( const QPoint pt, const QSize displaySize, const int capturePos,
                              const double scale, QPoint &ptCapture, QLine &lineOne );
    int TestAgainstRubberBands( QPoint pt, const QSize displaySize, QRect &rectRubberBand, QRect &rectROI,
                                const int capturePos, const double scale, QPoint &ptCapture );
    int EvalRectCapturePt( const QRect rectRubberBand, const QPoint ptAdj,
                           const int captureRadius, int &capturePos, QPoint &ptCapture );
    int EvalRulerCapturePt( const QLine lineOne, const QPoint ptAdj, const double scale,
                            const int captureRadius, int &capturePos, QPoint &ptCapture );

private:
    int AdjustPointFindLines( const QSize displaySize, const double scale, QLine &lineOne );
    int AdjustPointRubberBand( const QSize displaySize, QRect &rectRubberBand );
};

#endif // ROIADJUST_H
