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
The purpose of this project is to provide a hydrology and natural resource
science software tool that reduces the complexity of the analysis of data
and images in a way that is usable by commercial, academic, and other interested
parties. The goal is that at least some aspects of the tool will be useful
to everyone from middle school through doctoral level researches in field,
laboratory, and classroom settings.

**What is available. What is planned.**
While some aspects of the software are currently usable and have been used in
active field measurement and to perform scholarly academic work, the software is
very much a work in progress. For now, the main functionality that is operational
and mature enough for use is the measurement of water level in images when a
calibration target has been placed in the scene. That being said, software to
calculate image features suitable for use in common machine learing tasks, 
perform data fusion of the features with ground truth data from sources like the
USGS, and the actual creation of machine learning models exists. Results from a
study using these new, in development aspects of the software are in scholarly
review for [publication](https://hess.copernicus.org/preprints/hess-2020-575/)
in the HESS Journal and will be integrated into the tool in due course.

**Further into the future**
For now, the tool is most usable as a Windows 10 desktop application. The tool is
written so that it will be straightforward to create other application interfaces
including the following:

* Windows 10
* Debian installer
* Command line interface for Windows/Linux
* Web interface

Finally, while the tool is set up to consume hydrology and natural resources 
science data and images from wherever they are available, we hope to do a reference
application to capture images with an embedded computer that manages 24/7 capture
of images, transmission of images and data to a remote server, and onboard data 
analysis.

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
    * (Future) command line interface
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
3. ExifTool 11.88 - (system calls only) - [Artistic License](https://dev.perl.org/licenses/artistic.html)
4. ffmpeg 4.2.4 - (system calls only) - [LGPL 2.1](http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html)
5. Qt 5.15.2 (dynamically linked) - [LGPL 3.0](https://www.gnu.org/licenses/lgpl-3.0.en.html)

<a name="windev"></a>
## 3 Windows dev setup

<a name="linuxdev"></a>
## 4 Linux dev setup

<a name="tutor"></a>
## 5. Tutorials
