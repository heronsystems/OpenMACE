#-------------------------------------------------
#
# Project created by QtCreator 2017-06-27T15:16:17
#
#-------------------------------------------------

QT       -= core gui

TARGET = voropp
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++11


DEFINES += VOROPP_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += voropp.cpp \
        c_loops.cc \
        cell.cc \
        cmd_line.cc \
        common.cc \
        container.cc \
        container_prd.cc \
        pre_container.cc \
        unitcell.cc \
        v_base.cc \
        v_base_wl.cc \
        v_compute.cc \
        wall.cc \
        voro_index.cc

HEADERS += voropp.h\
        voropp_global.h \
        c_loops.hh \
        cell.hh \
        common.hh \
        config.hh \
        container.hh \
        container_prd.hh \
        pre_container.hh \
        rad_option.hh \
        unitcell.hh \
        v_base.hh \
        v_compute.hh \
        wall.hh \
        worklist.hh \
        voro_index.hh

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/voropp.lib release/voropp.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/voropp.lib debug/voropp.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)
