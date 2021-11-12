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
    cout << "Test mode: N/A" << endl;
    return 1;
#endif
}
