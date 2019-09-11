QT += core
QT -= gui

CONFIG += c++11

TARGET = TestMaps
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -O0

QMAKE_CXXFLAGS_WARN_ON += -Wno-unknown-pragmas

SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Copy Files
target.path = $$(MACE_ROOT)/bin
INSTALLS += target


#Necessary header includes
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/

unix {
INCLUDEPATH += /usr/include/boost
LIBS += -lm
LIBS += -lpthread
LIBS += -llz4
LIBS += -lX11
}

win32 {
INCLUDEPATH += $$(MACE_ROOT)/tools/boost_local
LIBS += -L $$(MACE_ROOT)/tools/boost_local
LIBS += -lgdi32
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

INCLUDEPATH += $$PWD/../maps
DEPENDPATH += $$PWD/../maps

unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann
unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann_s

INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
DEPENDPATH += $$PWD/../../tools/flann/src/cpp

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:unix:!macx: LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../planners/release/ -lplanners
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../planners/debug/ -lplanners
else:unix:!macx: LIBS += -L$$OUT_PWD/../planners/ -lplanners

INCLUDEPATH += $$PWD/../planners
DEPENDPATH += $$PWD/../planners

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../graphs/release/ -lgraphs
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../graphs/debug/ -lgraphs
else:unix:!macx: LIBS += -L$$OUT_PWD/../graphs/ -lgraphs

INCLUDEPATH += $$PWD/../graphs
DEPENDPATH += $$PWD/../graphs
