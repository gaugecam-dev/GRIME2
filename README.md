# GaugeCam GRIME2
GaugeCam Remote Image Manager v2 (Hydrology research tool)
http://gaugecam.org

**CONTENTS**
1. [ Introduction. ](#intro)
2. [ Usage. ](#usage)
3. [ Development. ](#develop)
4. [ Dev--Windows. ](#windev)
5. [ Dev--Linux. ](#linuxdev)
6. [ Tutorials. ](#tutor)

<a name="intro"></a>
## 1. Introduction

<a name="usage"></a>
## 2. Usage

<a name="develop"></a>
## 3. Development

**Preliminary notes**
The design goals for these libraries are to provide libraries to accomplish
tasks commonly needed in the Hydrology and Natural Resource Sciences. The idea
is to keep those libraries be kept distinct so they can be deployed with a
variety of user interfaces dependent on the end user community. We have started
with a graphical use interface (GUI) based on Qt (LGPL only), but plan to add
command line and web interfaces as time permits.

1. Hydrology and Natural Resource Sciences libraries
    * Machine vision
    * Data management
    * Machine learning
    * Etc.
2. User interface code
    * Desktop applications for Windows and Linux based on Qt
    * (Future) comman line interface
    * (Future) web-based GUI
    * (Future) Python wrapper
    
The GaugeCam team uses Qt Creator and the qmake system as their primary development
environment. We have considered switching to CMake, but legacy development practices
and other priorities precludes us from implementing that immediately.

**Prerequisites and licensing considerations**
The purpose of the GRIME2 libraries is to make them available for commercial and
non-commercial use: Free is in liberty and free as in beer. To that end, we have
released the code with the Apache 2.0 license. In addition, we have tried to chose
the libraries used to assure GRIME2 can be used commercially as long as there is
compliance with the Apache 2.0 license. Here are the libraries used by GRIME2:

1. OpenCV 4.5.1 (dynamically linked) - [Apache 2.0](https://github.com/opencv/opencv/blob/master/LICENSE)
2. Boost 1.74 - Boost Software License - [Version 1.0](https://www.boost.org/LICENSE_1_0.txt)
3. ExifTool - (system call only) - [Artistic License](https://dev.perl.org/licenses/artistic.html)


<a name="windev"></a>
## 3 Windows dev setup

<a name="linuxdev"></a>
## 4 Linux dev setup

<a name="tutor"></a>
## 5. Tutorials
