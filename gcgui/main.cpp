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

#include "mainwindow.h"
#include <QtWidgets/QApplication>

#ifdef TEST_JUNK
#undef TEST_JUNK
#include <opencv2/imgcodecs.hpp>
#include "../algorithms/findstaffgauge.h"
#endif

int main(int argc, char *argv[])
{
#ifndef TEST_JUNK
    QApplication a(argc, argv);

#ifndef win32
    a.setWindowIcon( QIcon( "./icons/coffee_logo.png" ) );
#endif

    MainWindow w;
    w.show();

    return a.exec();
#else
    FindStaffGauge find;
    cv::Mat img = imread( "/media/kchapman/Elements/unl/article_repo/trunk/unl_third_paper/StaffGaugeWithSymbol.png" );

    img = ~img;
    cvtColor( img, img, CV_BGR2GRAY );
    imwrite( "/var/tmp/water/gray_invert0.png", img );

    adaptiveThreshold( img, img, 255.0, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 51, 0.0 );
    imwrite( "/var/tmp/water/gray_invert_thresh0.png", img );

    Mat horizEdge;
    Sobel( img, horizEdge, CV_32FC1, 0, 1 );
    imwrite( "/var/tmp/water/horiz_edge0.tiff", horizEdge );

    Mat scratch;
    distanceTransform( img, scratch, DIST_L2, DIST_MASK_3 );
    imwrite( "/var/tmp/water/dist_transform0.tiff", scratch );

    double minVal, maxVal;
    minMaxIdx( scratch, &minVal, &maxVal );
    scratch.convertTo( scratch, CV_8UC1, 255.0 / ( maxVal - minVal ), -minVal );
    imwrite( "/var/tmp/water/dist_transform0_8u.png", scratch );

    // adaptiveThreshold( scratch, scratch, 255.0, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 101, -30.0 );
    threshold( scratch, scratch, 3.0, 255.0, THRESH_OTSU );
    imwrite( "/var/tmp/water/dist_transform0_8u_thresh.png", scratch );

    std::vector< double > tickLengths = { 35.0, 54.0, 65.0 };
    GC_STATUS retVal = find.Find( img, Point2d( 0.0, 24.0 ), -1.0, tickLengths );
    return GC_OK == retVal ? 0 : 1;
#endif
}
