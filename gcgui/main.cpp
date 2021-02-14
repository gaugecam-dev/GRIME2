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
#include "../algorithms/csvreader.h"
#include "../algorithms/timestampconvert.h"
#include "../algorithms/findanchor.h"
#include "../algorithms/metadata.h"
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
    MetaData meta;
    FindData data;
//    meta.WriteLineFindResult( "/media/kchapman/ThriveArchive/archive/utmb-feb/2021-02-01_17-35-23.304960_14d0fb24-0ae0-48ff-83af-8c06c76dc4d0/B6_0a83c097-4a48-4a99-8999-d8c9db382d0a/2021-02-01_17-38-07.739042_c9577d59-ee56-4ff1-a541-07a76e1abe27.png", data );
    GC_STATUS retVal = meta.WriteLineFindResult( "/home/kchapman/Projects/gaugecam/trunk/gcgui/config/2012_demo/05/NRmarshDN-12-05-31-23-00.jpg", data );
    return GC_OK == retVal ? 0 : 1;
#endif
}
