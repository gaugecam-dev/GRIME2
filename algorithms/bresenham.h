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
/** \file bresenham.h
 * @brief Include filt that holds a function to calculate all the points between
 *        two user specified end points
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2024, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef BRESENHAM_H
#define BRESENHAM_H

#include "log.h"
#include "gc_types.h"
#include <vector>
#include <opencv2/core.hpp>

using namespace gc;

/**
* @brief Standalone function to calculate all the points in a straight line between
*        two user specified end points
* @param pt0 Starting end point
* @param pt1 Ending end point
* @param linePts Vector of points on the line between pt0 and pt1
* @param maxPoints Maximum number of points to calculate
* @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
*/
GC_STATUS bresenham( const cv::Point pt0, const cv::Point pt1,
                     std::vector< cv::Point > &linePts, const size_t maxPoints = 999999999 )
{
    GC_STATUS retVal = pt0 == pt1 ? GC_ERR : GC_OK;

    if ( GC_OK != retVal )
    {
        FILE_LOG( logERROR ) << "[bresenham][" << __func__ << "] Line start and end points are the same";
        retVal = GC_ERR;
    }
    else
    {
        try
        {
            linePts.clear();

            int dx, dy, p, x, y;
            dx = pt1.x - pt0.x;
            dy = pt1.y - pt0.y;

            x = pt0.x;
            y = pt0.y;

            p = 2 * dy - dx;

            if ( abs( dx ) > abs( dy ) )
            {
                while ( x < pt1.x && linePts.size() < maxPoints )
                {
                    linePts.push_back( cv::Point( x, y ) );
                    if( 0 <= p )
                    {
                        y = y + 1;
                        p = p + 2 * dy - 2 * dx;
                    }
                    else
                    {
                        p = p + 2 * dy;
                    }
                    x = x + 1;
                }
            }
            else
            {
                while ( y > pt1.y && linePts.size() < maxPoints )
                {
                    linePts.push_back( cv::Point( x, y ) );
                    if( 0 <= p )
                    {
                        x = x + 1;
                        p = p + 2 * dx - 2 * dy;
                    }
                    else
                    {
                        p = p + 2 * dx;
                    }
                    y = y - 1;
                }
            }
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[bresenham][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}

#endif // BRESENHAM_H
