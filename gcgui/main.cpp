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

#ifndef TEST_JUNK
#define TEST_JUNK
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
    std::vector< double > tickLengths = { 35.0, 54.0, 65.0 };
    GC_STATUS retVal = find.Find( img, Point2d( 0.0, 24.0 ), -1.0, tickLengths );
    return GC_OK == retVal ? 0 : 1;
#endif
}
