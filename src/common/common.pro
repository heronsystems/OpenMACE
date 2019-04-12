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

SOURCES +=

HEADERS += common.h\
    enum_class_hash.h \
    publisher.h \
    class_forward.h \
    optional_parameter.h \
    pointer_collection.h \
    transmit_queue.h \
    thread_manager.h \
    chain inheritance.h \
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

