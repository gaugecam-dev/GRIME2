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

#include "roiadjust.h"
#include <cmath>

static double DistToLine( const double dX, const double dY,
                          const double dX1, const double dY1,
                          const double dX2, const double dY2 )
{
    double dNum = fabs( ( dX2 - dX1 ) * ( dY1 - dY ) - ( dX1 - dX ) * ( dY2 - dY1 ) );
    double dDenom = sqrt( ( dX2 - dX1 ) * ( dX2 - dX1 ) + ( dY2 - dY1 ) * ( dY2 - dY1 ) );
    return 0.0 == dDenom ? 0.0 : dNum / dDenom;
}

RoiAdjust::RoiAdjust()
{
}
int RoiAdjust::TestAgainstFindLines( const QPoint pt, const QSize displaySize, const int capturePos,
                                     const double scale, QPoint &ptCapture, QLine &lineOne )
{
    QPoint ptTemp;
    ptTemp.setX( qRound( static_cast< double >( pt.x() ) / scale ) );
    ptTemp.setY( qRound( static_cast< double >( pt.y() ) / scale ) );
    switch ( capturePos )
    {
        case 1: lineOne.setP1( ptTemp ); break;
        case 2: lineOne.setP2( ptTemp ); break;
        case 5:
            lineOne.setP1( lineOne.p1() + ( ptTemp - ptCapture ) );
            lineOne.setP2( lineOne.p2() + ( ptTemp - ptCapture ) );
            break;
        default: break;
    }
    AdjustPointFindLines( displaySize, scale, lineOne );
    ptCapture = ptTemp;

    return 0;
}
int RoiAdjust::AdjustPointFindLines( const QSize displaySize, const double scale, QLine &lineOne )
{
    int nWidth = qRound( static_cast< double >( displaySize.width() ) / scale ) - 5;
    int nHeight = qRound( static_cast< double >( displaySize.height() ) / scale ) - 5;
    QPoint ptLine1Pt1( lineOne.p1().x(), lineOne.p1().y() );
    QPoint ptLine1Pt2( lineOne.p2().x(), lineOne.p2().y() );
    if ( 5 > ptLine1Pt1.x() ) ptLine1Pt1.setX( 5 );
    if ( 5 > ptLine1Pt2.x() ) ptLine1Pt2.setX( 5 );
    if ( 5 > ptLine1Pt1.y() ) ptLine1Pt1.setY( 5 );
    if ( 5 > ptLine1Pt2.y() ) ptLine1Pt2.setY( 5 );
    if ( nWidth <= ptLine1Pt1.x() ) ptLine1Pt1.setX( nWidth - 1 );
    if ( nWidth <= ptLine1Pt2.x() ) ptLine1Pt2.setX( nWidth - 1 );
    if ( nHeight <= ptLine1Pt1.y() ) ptLine1Pt1.setY( nHeight - 1 );
    if ( nHeight <= ptLine1Pt2.y() ) ptLine1Pt2.setY( nHeight - 1 );
    lineOne.setP1( ptLine1Pt1 );
    lineOne.setP2( ptLine1Pt2 );

    return 0;
}
int RoiAdjust::TestAgainstRubberBands( QPoint pt, const QSize displaySize, QRect &rectRubberBand, QRect &rectROI,
                                       const int capturePos, const double scale, QPoint &ptCapture )
{
    switch ( capturePos )
    {
        case 1:
            rectRubberBand.setLeft( pt.x() );
            rectRubberBand.setTop( pt.y() );
            break;
        case 2:
            rectRubberBand.setTop( pt.y() );
            break;
        case 4:
            rectRubberBand.setRight( pt.x() );
            rectRubberBand.setTop( pt.y() );
            break;
        case 8: rectRubberBand.setLeft( pt.x() ); break;
        case 16: rectRubberBand.setRight( pt.x() ); break;
        case 32:
            rectRubberBand.setLeft( pt.x() );
            rectRubberBand.setBottom( pt.y() );
            break;
        case 64:
            rectRubberBand.setBottom( pt.y() );
            break;
        case 128:
            rectRubberBand.setRight( pt.x() );
            rectRubberBand.setBottom( pt.y() );
            break;
        case 255: rectRubberBand.setCoords(
            rectRubberBand.left() - ptCapture.x() + pt.x(),
            rectRubberBand.top() - ptCapture.y() + pt.y(),
            rectRubberBand.right() - ptCapture.x() + pt.x(),
            rectRubberBand.bottom() - ptCapture.y() + pt.y() );
            break;
        default: break;
    }
    AdjustPointRubberBand( displaySize, rectRubberBand );
    rectROI.setLeft( qRound( static_cast< double >( rectRubberBand.left() ) / scale ) );
    rectROI.setTop( qRound( static_cast< double >( rectRubberBand.top() ) / scale ) );
    rectROI.setRight( qRound( static_cast< double >( rectRubberBand.right() ) / scale ) );
    rectROI.setBottom( qRound( static_cast< double >( rectRubberBand.bottom() ) / scale ) );
    ptCapture = pt;

    return 0;
}
int RoiAdjust::AdjustPointRubberBand( const QSize displaySize, QRect &rectRubberBand )
{
    if ( 5 > rectRubberBand.width() ) rectRubberBand.setRight( rectRubberBand.x() + 5 );
    if ( 5 > rectRubberBand.height() ) rectRubberBand.setBottom( rectRubberBand.y() + 5 );
    if ( 0 > rectRubberBand.x() ) rectRubberBand.setLeft( 0 );
    if ( 0 > rectRubberBand.y() ) rectRubberBand.setTop( 0 );
    if ( displaySize.width() < rectRubberBand.left() + rectRubberBand.width() )
            rectRubberBand.setRight( displaySize.width() - 1 );
    if ( displaySize.height() < rectRubberBand.top() + rectRubberBand.height() )
            rectRubberBand.setBottom( displaySize.height() - 1 );
    if ( 5 > rectRubberBand.width() ) rectRubberBand.setLeft( rectRubberBand.right() - 5 );
    if ( 5 > rectRubberBand.height() ) rectRubberBand.setTop( rectRubberBand.bottom() - 5 );

    return 0;
}
int RoiAdjust::EvalRectCapturePt( const QRect rectRubberBand, const QPoint ptAdj,
                                  const int captureRadius, int &capturePos, QPoint &ptCapture )
{
    int nX = ptAdj.x();
    int nY = ptAdj.y();
    ptCapture.setX( nX );
    ptCapture.setY( nY );
    if ( abs( nX - rectRubberBand.left() ) < captureRadius )
    {
        if ( abs( nY - rectRubberBand.top() ) < captureRadius )               capturePos = 1;
        else if ( abs( nY - rectRubberBand.bottom() ) < captureRadius )       capturePos = 32;
        else if ( nY > rectRubberBand.top() && nY < rectRubberBand.bottom() ) capturePos = 8;
        else capturePos = 0;
    }
    else if ( abs( nX - rectRubberBand.right() ) < captureRadius )
    {
        if ( abs( nY - rectRubberBand.top() ) < captureRadius )               capturePos = 4;
        else if ( abs( nY - rectRubberBand.bottom() ) < captureRadius )       capturePos = 128;
        else if ( nY > rectRubberBand.top() && nY < rectRubberBand.bottom() ) capturePos = 16;
        else capturePos = 0;
    }
    else if ( nX > rectRubberBand.left() && nX < rectRubberBand.right() )
    {
        if ( abs ( nY - rectRubberBand.top() ) < captureRadius )               capturePos = 2;
        else if ( abs ( nY - rectRubberBand.bottom() ) < captureRadius )       capturePos = 64;
        else if ( nY > rectRubberBand.top() && nY < rectRubberBand.bottom() )   capturePos = 255;
        else capturePos = 0;
    }

    return 0;
}
int RoiAdjust::EvalRulerCapturePt( const QLine lineOne, const QPoint ptAdj, const double scale,
                                   const int captureRadius, int &capturePos, QPoint &ptCapture )
{
    capturePos = 0;
    int nX = qRound( static_cast< double >( ptAdj.x() ) / scale + 0.5 );
    int nY = qRound( static_cast< double >( ptAdj.y() ) / scale + 0.5 );
    ptCapture.setX( nX );
    ptCapture.setY( nY );
    if ( abs( nX - lineOne.p1().x() ) < captureRadius &&
         abs( nY - lineOne.p1().y() ) < captureRadius ) capturePos = 1;
    else if ( abs( nX - lineOne.p2().x() ) < captureRadius &&
              abs( nY - lineOne.p2().y() ) < captureRadius ) capturePos = 2;
    else
    {
        double dDist = DistToLine( static_cast< double >( nX ), static_cast< double >( nY ),
                                   static_cast< double >( lineOne.p1().x() ),
                                   static_cast< double >( lineOne.p1().y() ),
                                   static_cast< double >( lineOne.p2().x() ),
                                   static_cast< double >( lineOne.p2().y() ) );
        if ( 10 > qRound( dDist + 0.5 ) ) capturePos = 255;
    }
    return 0;
}
int RoiAdjust::EvalPolyCapturePt( const LineSearchPoly guiPoly, const QPoint ptAdj, const double scale,
                                  const int captureRadius, int &capturePos, QPoint &ptCapture )
{
    capturePos = 0;
    int nX = qRound( static_cast< double >( ptAdj.x() ) / scale );
    int nY = qRound( static_cast< double >( ptAdj.y() ) / scale );
    ptCapture.setX( nX );
    ptCapture.setY( nY );

    if ( abs( nX - guiPoly.lftTop.x() ) < captureRadius &&  abs( nY - guiPoly.lftTop.y() ) < captureRadius )
    {
        capturePos = 1;
    }
    else if ( abs( nX - guiPoly.rgtTop.x() ) < captureRadius &&  abs( nY - guiPoly.rgtTop.y() ) < captureRadius )
    {
        capturePos = 4;
    }
    else if ( abs( nX - guiPoly.lftBot.x() ) < captureRadius &&  abs( nY - guiPoly.lftBot.y() ) < captureRadius )
    {
        capturePos = 32;
    }
    else if ( abs( nX - guiPoly.rgtBot.x() ) < captureRadius &&  abs( nY - guiPoly.rgtBot.y() ) < captureRadius )
    {
        capturePos = 128;
    }
    else if ( nX > guiPoly.lftTop.x() && nX < guiPoly.rgtTop.x() &&
              nX > guiPoly.lftBot.x() && nX < guiPoly.rgtBot.x() &&
              nY > guiPoly.lftTop.y() && nY < guiPoly.lftBot.y() &&
              nY > guiPoly.rgtTop.y() && nY < guiPoly.rgtBot.y() )
    {
        capturePos = 255;
    }
     return 0;
}
int RoiAdjust::TestAgainstPoly( QPoint pt, const QSize displaySize, LineSearchPoly &guiPoly,
                                const int capturePos, const double scale, QPoint &ptCapture )
{
    int ret = 0;

    bool doAdjust = true;

    QPoint ptTemp;
    ptTemp.setX( qRound( static_cast< double >( pt.x() ) / scale ) );
    ptTemp.setY( qRound( static_cast< double >( pt.y() ) / scale ) );

    switch ( capturePos )
    {
        case 1: guiPoly.lftTop = ptTemp; break;
        case 4: guiPoly.rgtTop = ptTemp;  break;
        case 32: guiPoly.lftBot = ptTemp; break;
        case 128:  guiPoly.rgtBot = ptTemp; break;
        case 255:
            guiPoly.lftTop = guiPoly.lftTop + ( ptTemp - ptCapture );
            guiPoly.rgtTop = guiPoly.rgtTop + ( ptTemp - ptCapture );
            guiPoly.lftBot = guiPoly.lftBot + ( ptTemp - ptCapture );
            guiPoly.rgtBot = guiPoly.rgtBot + ( ptTemp - ptCapture );
            break;
        default:
            doAdjust = false;
            break;
    }

    if ( doAdjust )
    {
        ret = AdjustPointPoly( displaySize, scale, guiPoly );
    }

    ptCapture = ptTemp;

    return ret;
}
int RoiAdjust::AdjustPointPoly( const QSize displaySize, double const scale, LineSearchPoly &guiPoly )
{
    int ret = 0;

    int nWidth = qRound( static_cast< double >( displaySize.width() ) / scale ) - 1;
    int nHeight = qRound( static_cast< double >( displaySize.height() ) / scale ) - 1;

    if ( 0 > guiPoly.lftTop.x() ) guiPoly.lftTop.setX( 0 );
    if ( 0 > guiPoly.lftBot.x() ) guiPoly.lftBot.setX( 0 );
    if ( 0 > guiPoly.rgtTop.x() ) guiPoly.rgtTop.setX( 0 );
    if ( 0 > guiPoly.rgtBot.x() ) guiPoly.rgtBot.setX( 0 );
    if ( nWidth < guiPoly.lftTop.x() ) guiPoly.lftTop.setX( nWidth - 1 );
    if ( nWidth < guiPoly.lftBot.x() ) guiPoly.lftBot.setX( nWidth - 1 );
    if ( nWidth < guiPoly.rgtTop.x() ) guiPoly.rgtTop.setX( nWidth - 1 );
    if ( nWidth < guiPoly.rgtBot.x() ) guiPoly.rgtBot.setX( nWidth - 1 );

    if ( 0 > guiPoly.lftTop.y() ) guiPoly.lftTop.setY( 0 );
    if ( 0 > guiPoly.lftBot.y() ) guiPoly.lftBot.setY( 0 );
    if ( 0 > guiPoly.rgtTop.y() ) guiPoly.rgtTop.setY( 0 );
    if ( 0 > guiPoly.rgtBot.y() ) guiPoly.rgtBot.setY( 0 );
    if ( nHeight < guiPoly.lftTop.y() ) guiPoly.lftTop.setY( nHeight - 1 );
    if ( nHeight < guiPoly.lftBot.y() ) guiPoly.lftBot.setY( nHeight - 1 );
    if ( nHeight < guiPoly.rgtTop.y() ) guiPoly.rgtTop.setY( nHeight - 1 );
    if ( nHeight < guiPoly.rgtBot.y() ) guiPoly.rgtBot.setY( nHeight - 1 );

    return ret;
}
