#-------------------------------------------------
#
# Project created by QtCreator 2017-11-17T09:45:10
#
#-------------------------------------------------

QT += serialport
QT       -= gui

QMAKE_CXXFLAGS += -std=c++14

TARGET = DigiMesh
TEMPLATE = lib

DEFINES += DIGIMESH_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    serial_link.cpp \
    digimesh_radio.cpp

HEADERS += \
    ATData/I_AT_data.h \
    ATData/index.h \
    ATData/integer.h \
    ATData/message.h \
    ATData/node_discovery.h \
    ATData/string.h \
    ATData/void.h \
    frame-persistance/behaviors/base.h \
    frame-persistance/behaviors/collect-and-timeout.h \
    frame-persistance/behaviors/index.h \
    frame-persistance/behaviors/shutdown-first-response.h \
    frame-persistance/types/collect-and-timeout.h \
    frame-persistance/types/index.h \
    frame-persistance/types/shutdown-first-response.h \
    callback.h \
    DigiMesh_global.h \
    digimesh_radio.h \
    i_link_events.h \
    math_helper.h \
    serial_configuration.h \
    serial_link.h \
    timer.h \
    ATData/transmit_status.h


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/DigiMesh.lib release/DigiMesh.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/DigiMesh.lib debug/DigiMesh.dll
INSTALLS += lib

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon
