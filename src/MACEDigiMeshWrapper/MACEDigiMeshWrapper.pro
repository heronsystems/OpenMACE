#-------------------------------------------------
#
# Project created by QtCreator 2017-11-17T10:10:19
#
#-------------------------------------------------
QT += serialport
QT       -= gui

QMAKE_CXXFLAGS += -std=c++14

TARGET = MACEDigiMeshWrapper
TEMPLATE = lib

DEFINES += MACEDIGIMESHWRAPPER_LIBRARY

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
    component.cpp \
    interop_component.cpp \
    interop.cpp

HEADERS +=\
        macewrapper_global.h \
    mace_digimesh_wrapper.h \
    component.h \
    interop_component.h \
    interop.h \
    resource.h

#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DigiMesh/release/ -lDigiMesh
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DigiMesh/debug/ -lDigiMesh
else:unix: LIBS += -L$$OUT_PWD/../DigiMesh/ -lDigiMesh

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/MACEDigiMeshWrapper.lib release/MACEDigiMeshWrapper.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/MACEDigiMeshWrapper.lib debug/MACEDigiMeshWrapper.dll
INSTALLS += lib

INCLUDEPATH += $$PWD/../digi_common
DEPENDPATH += $$PWD/../digi_common

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon
