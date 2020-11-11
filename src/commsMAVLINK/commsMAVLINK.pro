#-------------------------------------------------
#
# Project created by QtCreator 2017-03-04T19:33:05
#
#-------------------------------------------------
QT += serialport
QT += network
QT      += core
QT      -= gui

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


TARGET = commsMAVLINK
TEMPLATE = lib

DEFINES += COMMSMAVLINK_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += comms_mavlink.cpp

HEADERS += comms_mavlink.h\
        commsmavlink_global.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../spdlog/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/commsMAVLINK.lib release/commsMAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/commsMAVLINK.lib debug/commsMAVLINK.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

unix {
    exists($$(ROS_ROOT_DIR)/lib/) {

      DEFINES += ROS_EXISTS
      INCLUDEPATH += $$(ROS_ROOT_DIR)/include
      INCLUDEPATH += $$(ROS_ROOT_DIR)/lib
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomath
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomap

    } else {
      message("ROS root" path has not been detected...)
      INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
      LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath

      # Octomap Warning suppression:
      QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include
    }
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
win32:INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include

