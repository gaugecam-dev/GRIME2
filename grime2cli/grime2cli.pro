TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += BOOST_ALL_NO_LIB BOOST_BIND_GLOBAL_PLACEHOLDERS

win32 {
    DEFINES += NOMINMAX
    DEFINES += WIN32_LEAN_AND_MEAN
    DEFINES += _WIN32_WINNT=0x0501
    OPENCV_INCLUDES = c:/opencv/opencv_4.10.0/include
    OPENCV_LIBS = C:/opencv/opencv_4.10.0/x64/vc19/lib
    BOOST_INCLUDES = C:/Boost/boost_1_86/include
    BOOST_LIBS = C:/Boost/boost_1_86/lib
}

SOURCES += \
    ../algorithms/animate.cpp \
    ../algorithms/calibexecutive.cpp \
    ../algorithms/caliboctagon.cpp \
    ../algorithms/findline.cpp \
    ../algorithms/gifanim/gifanim.cpp \
    ../algorithms/metadata.cpp \
    ../algorithms/searchlines.cpp \
    ../algorithms/octagonsearch.cpp \
    ../algorithms/visapp.cpp \
    main.cpp

HEADERS += \
    ../algorithms/animate.h \
    ../algorithms/bresenham.h \
    ../algorithms/calibexecutive.h \
    ../algorithms/caliboctagon.h \
    ../algorithms/csvreader.h \
    ../algorithms/findline.h \
    ../algorithms/gifanim/gifanim.h \
    ../algorithms/gc_types.h \
    ../algorithms/labelroi.h \
    ../algorithms/log.h \
    ../algorithms/metadata.h \
    ../algorithms/timestampconvert.h \
    ../algorithms/searchlines.h \
    ../algorithms/octagonsearch.h \
    ../algorithms/visapp.h \
    ../gcgui/wincmd.h \
    arghandler.h \
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
        LIBS += -lopencv_core4100d \
                -lopencv_imgproc4100d \
                -lopencv_imgcodecs4100d \
                -lopencv_videoio4100d \
                -lopencv_video4100d \
                -lopencv_calib3d4100d \
                -llibboost_date_time-vc143-mt-gd-x32-1_86 \
                -llibboost_system-vc143-mt-gd-x32-1_86 \
                -llibboost_chrono-vc143-mt-gd-x32-1_86 \
                -ladvapi32
    } else {
        LIBS += -lopencv_core4100 \
                -lopencv_imgproc4100 \
                -lopencv_imgcodecs4100 \
                -lopencv_videoio4100 \
                -lopencv_video4100 \
                -lopencv_calib3d4100 \
                -llibboost_date_time-vc143-mt-x64-1_86 \
                -llibboost_system-vc143-mt-x64-1_86 \
                -llibboost_chrono-vc143-mt-x64-1_86 \
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
    Makefile \
    NOTICE \
    article_three_batch \
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
