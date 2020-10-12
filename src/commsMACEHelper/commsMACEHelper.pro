#-------------------------------------------------
#
# Project created by QtCreator 2017-05-01T13:52:04
#
#-------------------------------------------------

QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = commsMACEHelper
TEMPLATE = lib

DEFINES += COMMSMACEHELPER_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += comms_mace_helper.cpp

HEADERS += comms_mace_helper.h\
        commsmacehelper_global.h

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3
# Octomap Warning suppression:
QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/commsMACEHelper.lib release/commsMACEHelper.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/commsMACEHelper.lib debug/commsMACEHelper.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/release/ -lMACEDigiMeshWrapper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/debug/ -lMACEDigiMeshWrapper
else:unix:!macx: LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/ -lMACEDigiMeshWrapper

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/release/ -lcommsMACE
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/debug/ -lcommsMACE
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMACE/ -lcommsMACE

