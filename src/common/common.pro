#-------------------------------------------------
#
# Project created by QtCreator 2016-09-02T13:57:03
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = common
TEMPLATE = lib

DEFINES += COMMON_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES +=

HEADERS += common.h\
    enum_class_hash.h \
    publisher.h \
    class_forward.h \
    optional_parameter.h \
    pointer_collection.h \
    transmit_queue.h \
    thread_manager.h \
    chain_inheritance.h \
    object_int_tuple.h \
    background_tasks.h \
    watchdog.h \
    numerical_analysis.h \
    logging/termcolor.hpp \
    logging/macelog.h


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$(MACE_ROOT)/include

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/common.lib release/common.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/common.lib debug/common.dll
INSTALLS += lib
