TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += BOOST_ALL_NO_LIB BOOST_BIND_GLOBAL_PLACEHOLDERS
CONFIG += c++11

win32 {
    DEFINES += NOMINMAX
    DEFINES += WIN32_LEAN_AND_MEAN
    DEFINES += _WIN32_WINNT=0x0501
    OPENCV_INCLUDES = c:/opencv/opencv_451/include
    OPENCV_LIBS = C:/opencv/opencv_451/x64/lib/vc19
    BOOST_INCLUDES = C:/local/boost_1_74_0
    BOOST_LIBS = C:/local/boost_1_74_0/stage/lib
}

SOURCES += \
        ../algorithms/calib.cpp \
        ../algorithms/findcalibgrid.cpp \
        ../algorithms/findline.cpp \
        ../algorithms/metadata.cpp \
        ../algorithms/visapp.cpp \
        main.cpp

HEADERS += \
    ../algorithms/bresenham.h \
    ../algorithms/calib.h \
    ../algorithms/csvreader.h \
    ../algorithms/findcalibgrid.h \
    ../algorithms/findline.h \
    ../algorithms/gc_types.h \
    ../algorithms/log.h \
    ../algorithms/metadata.h \
    ../algorithms/timestampconvert.h \
    ../algorithms/visapp.h \
    ../gcgui/wincmd.h \
    arghandler.h

unix:!macx {
    INCLUDEPATH +=  /usr/local/include \
                    /usr/local/include/opencv4

    LIBS += -L/usr/local/lib \
            -lopencv_core \
            -lopencv_imgproc \
            -lopencv_imgcodecs \
            -lopencv_calib3d \
            -lopencv_videoio \
            -lopencv_video \
            -lboost_date_time \
            -lboost_system \
            -lboost_filesystem \
            -lboost_chrono
}
else {
    INCLUDEPATH += $$BOOST_INCLUDES \
                   $$OPENCV_INCLUDES \
                   ../libs/imgproc \
                   ../utility
    DEPENDPATH += $$BOOST_INCLUDES \
                  $$BOOST_LIBS \
                  $$OPENCV_INCLUDES \
                  $$OPENCV_LIBS

    LIBS += -L$$BOOST_LIBS \
            -L$$OPENCV_LIBS

    CONFIG(debug, debug|release) {
        LIBS += -lopencv_core451d \
                -lopencv_imgproc451d \
                -lopencv_imgcodecs451d \
                -lopencv_videoio451d \
                -lopencv_video451d \
                -lopencv_calib3d451d \
                -llibboost_filesystem-vc142-mt-gd-x64-1_74 \
                -llibboost_date_time-vc142-mt-gd-x64-1_74 \
                -llibboost_system-vc142-mt-gd-x64-1_74 \
                -llibboost_chrono-vc142-mt-gd-x64-1_74 \
                -ladvapi32
    } else {
        LIBS += -lopencv_core451 \
                -lopencv_imgproc451 \
                -lopencv_imgcodecs451 \
                -lopencv_videoio451 \
                -lopencv_video451 \
                -lopencv_calib3d451 \
                -llibboost_filesystem-vc142-mt-x64-1_74 \
                -llibboost_date_time-vc142-mt-x64-1_74 \
                -llibboost_system-vc142-mt-x64-1_74 \
                -llibboost_chrono-vc142-mt-x64-1_74 \
                -ladvapi32
    }
}

# copies the given files to the destination directory
# OTHER_FILES += $$PWD/../python/graphserver/dist/graphserver $$PWD/../python/graphserver/dist/graphserver.ui
# defineTest(copyToDest) {
#     files = $$1
#     dir = $$2
#     # replace slashes in destination path for Windows
#     win32:dir ~= s,/,\\,g

#     for(file, files) {
#         # replace slashes in source path for Windows
#         win32:file ~= s,/,\\,g

#         QMAKE_POST_LINK += $$QMAKE_COPY $$shell_quote($$file) $$shell_quote($$dir) $$escape_expand(\\n\\t)
#     }

#     export(QMAKE_POST_LINK)
# }

#copyToDest($$OTHER_FILES, $$OUT_PWD/)

RESOURCES += \
    resources.qrc

DISTFILES += \
    ../algorithms/Doxyfile \
    LICENSE \
    NOTICE \
    config/calib.json \
    config/calib_result.png \
    config/calibration_target_world_coordinates.csv \
    config/settings.cfg \
    config/settingsWin.cfg \
    docs/Background_installation_guideline.pdf \
    docs/boost_license.txt \
    docs/lgpl_license.txt \
    docs/perl_artistic_license.txt \
    docs/release_notes.html \
    LICENSE \
    NOTICE \
    grime2installer.nsi
