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

#include "log.h"
#include "findanchor.h"
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#ifdef DEBUG_FIND_ANCHOR
#undef DEBUG_FIND_ANCHOR
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>
#ifdef WIN32
static string DEBUG_FOLDER = "c:/gaugecam/debug/find_anchor/";
#else
static string DEBUG_FOLDER = "/var/tmp/gaugecam/find_anchor/";
#endif
#endif

namespace gc
{

FindAnchor::FindAnchor() :
    rectVert( Rect( -1, -1, -1, -1 ) ),
    rectHoriz( Rect( -1, -1, -1, -1 ) ),
    morphCountVert( -1 ),
    morphCountHoriz( -1 ),
    isDarkSparseVert( true ),
    isDarkSparseHoriz( true ),
    angleRef( -99999.0 ),
    offsetRef( Point( -9999999, -9999999 ) ),
    modelRect( Rect( -1, -1, -1, -1 ) )
{
#ifdef DEBUG_FIND_ANCHOR
    if ( !boost::filesystem::exists( DEBUG_FOLDER ) )
    {
        bool bOk = boost::filesystem::create_directories( DEBUG_FOLDER );
        if ( bOk )
        {
            FILE_LOG( logINFO ) << "Create find anchor debug folder: " << DEBUG_FOLDER;
        }
        else
        {
            FILE_LOG( logWARNING ) << "Could NOT create find anchor debug folder: " << DEBUG_FOLDER;
        }
    }
#endif
}
void FindAnchor::clear()
{
    rectVert = Rect( -1, -1, -1, -1 );
    rectHoriz = Rect( -1, -1, -1, -1 );
    morphCountVert = -1;
    morphCountHoriz = -1;
    isDarkSparseVert = true;
    isDarkSparseHoriz = true;
    angleRef = -99999.0;
    offsetRef = Point( -9999999, -9999999 );
    modelRect = Rect( -1, -1, -1, -1 ),
    rotModelSet.clear();
    matProbSpace = Mat();
}
bool FindAnchor::isInitializedVertHoriz()
{
    return ( 0 > rectVert.x || 0 > rectHoriz.x ||
             0 > morphCountVert || 0 > morphCountHoriz ||
             -360.0 > angleRef || -999999 > offsetRef.x ) ? false : true;
}
bool FindAnchor::isInitializedModel()
{
    return ( rotModelSet.empty() || matProbSpace.empty() ||
             0 > modelRect.x ) ? false : true;
}
GC_STATUS FindAnchor::SetRef( const string imgFilepath, const Rect &modelROI )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        Mat imgRef = imread( imgFilepath, IMREAD_COLOR );
        if ( imgRef.empty() )
        {
            FILE_LOG( logERROR ) << "[VisAppFeats::SetAnchorRef] could not read reference image " << imgFilepath;
            retVal = GC_ERR;
        }
        else
        {
            modelRefImageFilepath = imgFilepath;
            retVal = SetRef( imgRef, modelROI );
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::SetRef] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindAnchor::SetRef( const Mat &img, const Rect &modelROI  )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( img.empty() || modelROI.empty() )
        {
            FILE_LOG( logERROR ) << "[FindAnchor::SetRef] Cannot set reference with empty source image or model";
            retVal = GC_ERR;
        }
        else if ( modelROI.width + 50 > img.cols || modelROI.height + 50 > img.rows )
        {
            FILE_LOG( logERROR ) << "[FindAnchor::SetRef] Model (ref image) must be at least"
                                    " 20 pixels less in both dimensions than search image";
            retVal = GC_ERR;
        }
        else
        {
            clear();
            Mat scratch;
            if ( CV_8UC3 == img.type() )
                cvtColor( img, scratch, COLOR_BGR2GRAY );
            else
                scratch = img.clone();

#ifdef DEBUG_FIND_ANCHOR
            imwrite( DEBUG_FOLDER + "non_rotated_model.png", scratch );
#endif

            Mat rotScratch( scratch.size() * 2, CV_8UC1 );
            GaussianBlur( scratch, scratch, Size( 5, 5 ), 3.0 );
            Point2d ptCenter = Point2d( static_cast< double >( scratch.cols ) / 2.0,
                                        static_cast< double >( scratch.rows ) / 2.0 );

            for ( int i = -15; i <= 15; ++i )
            {
                retVal = RotateImage( scratch, rotScratch, ptCenter, static_cast< double >( i ) / 2.0 );
                if ( GC_OK != retVal )
                    break;
                rotModelSet.push_back( RotatedModel( rotScratch( modelROI ), static_cast< double >( i ) / 2.0 ) );
#ifdef DEBUG_FIND_ANCHOR
                putText( rotScratch, to_string( static_cast< double >( i ) / 2.0 ), Point( 10, 50 ), FONT_HERSHEY_PLAIN, 3.0, Scalar( 255 ), 3 );
                imwrite( DEBUG_FOLDER + "rotate_" + to_string( i + 115 ) + ".png", rotScratch( modelROI ) );
#endif
            }
            if ( GC_OK == retVal )
            {
                modelRect = modelROI;
                matProbSpace = Mat();
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::SetRef] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindAnchor::SetRef( const Mat &img, const vector< Point > regionV, const vector< Point > regionH, const bool darkSparseV,
                              const bool darkSparseH, const int morphCountV, const int morphCountH )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        clear();
        rectVert = boundingRect( regionV );
        rectHoriz = boundingRect( regionH );
        isDarkSparseVert = darkSparseV;
        isDarkSparseHoriz = darkSparseH;
        morphCountVert = morphCountV;
        morphCountHoriz = morphCountH;
        retVal = Find( img, angleRef, offsetRef );
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::SetRef] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS FindAnchor::Find( const Mat &img, double &angle, Point &offset )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( isInitializedVertHoriz() )
        {
            FILE_LOG( logERROR ) << "[FindAnchor::Find] Find based on horizontal and vertical edge intersection not yet implemented";
            retVal = GC_ERR;

            if ( GC_OK == retVal )
            {
                Point ptVa, ptVb, ptHa, ptHb;
                retVal = FindVert( img( rectVert ), ptVa, ptVb );
                if ( GC_OK == retVal )
                {
                    retVal = FindHoriz( img( rectHoriz ), ptHa, ptHb );
                }
                angle = -9999999.0;         // TODO: Remove when implemented
                offset = Point( -1, -1 );   // TODO: Remove when implemented
            }
        }
        else if ( isInitializedModel() )
        {
            retVal = FindModel( img, angle, offset );
#ifdef DEBUG_FIND_ANCHOR
            FILE_LOG( logDEBUG ) << "Anchor angle=" << angle << " x=" << offset.x << " y=" << offset.y;
#endif
        }
        else
        {
            FILE_LOG( logERROR ) << "[FindAnchor::Find] No find reference defined";
            retVal = GC_ERR;
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::Find] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindAnchor::CalcMoveModel( const cv::Mat &img, cv::Point &ptOrig, cv::Point &ptMove, double &angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( rotModelSet.empty() )
        {
            FILE_LOG( logERROR ) << "[FindAnchor::CalcMoveModel] No find reference defined";
            retVal = GC_ERR;
        }
        else
        {
            Mat scratch;
            if ( CV_8UC3 == img.type() )
                cvtColor( img, scratch, COLOR_BGR2GRAY );
            else
                scratch = img.clone();

            GaussianBlur( img, scratch, Size( 5, 5 ), 3.0 );

            matProbSpace = Mat();
            double maxScore = -9999999.0;
            ptOrig = Point( modelRect.x, modelRect.y );
            for ( size_t i = 0; i < rotModelSet.size(); ++i )
            {
                matchTemplate( scratch, rotModelSet[ i ].model, matProbSpace, TM_CCOEFF_NORMED );
                minMaxLoc( matProbSpace, nullptr, &rotModelSet[ i ].score, nullptr, &rotModelSet[ i ].offset );
                if ( rotModelSet[ i ].score > maxScore )
                {
                    ptMove = Point( modelRect.x, modelRect.y );
                    maxScore = rotModelSet[ i ].score;
                    angle = rotModelSet[ i ].angle;
                }
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::CalcMoveModel] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindAnchor::FindModel( const Mat &img, double &angle, cv::Point &offset )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        double maxScore = -9999999.0;
        angle = -99999.0;
        offset = Point( -1, -1 );
        for ( size_t i = 0; i < rotModelSet.size(); ++i )
        {
            matchTemplate( img, rotModelSet[ i ].model, matProbSpace, TM_CCORR_NORMED );
            minMaxLoc( matProbSpace, nullptr, &rotModelSet[ i ].score, nullptr, &rotModelSet[ i ].offset );
            if ( rotModelSet[ i ].score > maxScore )
            {
                angle = rotModelSet[ i ].angle;
                offset = rotModelSet[ i ].offset - Point( modelRect.x, modelRect.y );
                maxScore = rotModelSet[ i ].score;
            }
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::FindHoriz] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindAnchor::FindHoriz( const Mat &img, Point &ptA, Point &ptB )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( isDarkSparseHoriz )
        {
            Mat scratch;
            erode( img, scratch, morphCountHoriz );
            ptA = ptB = Point( -1, -1 );    // TODO: remove when implemented
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::FindHoriz] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindAnchor::FindVert( const Mat &img, Point &ptA, Point &ptB )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( isDarkSparseVert )
        {
            Mat scratch;
            erode( img, scratch, morphCountVert );
            ptA = ptB = Point( -1, -1 );    // TODO: remove when implemented
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[FindAnchor::FindVert] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS FindAnchor::RotateImage( const Mat &src, Mat &dst, const Point2d ptCenter, const double angle )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        // Point2d ptCenter = Point2d( static_cast< double >( src.cols ) / 2.0, static_cast< double >( src.rows ) / 2.0 );
        Mat matRotMatrix = getRotationMatrix2D( ptCenter, angle, 1.0 );
        // cout << matRotMatrix << endl;
        warpAffine( src, dst, matRotMatrix, dst.size(), INTER_CUBIC );
    }
    catch( exception &e )
    {
        FILE_LOG( logERROR ) << "[" << __func__ << "][FindCalibGrid::RotateImage] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}

} //namespace gc
