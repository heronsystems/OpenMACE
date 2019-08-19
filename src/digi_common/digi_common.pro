#-------------------------------------------------
#
# Project created by QtCreator 2017-12-01T09:48:26
#
#-------------------------------------------------

QT       -= core gui

TARGET = digi_common
TEMPLATE = lib

DEFINES += DIGI_COMMON_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES +=

HEADERS += \
    digi_mesh_baud_rates.h \
    transmit_status_types.h \
    discovery_status_types.h


#copydata.commands = $(MKDIR) $$PWD/../include ; $(COPY_DIR) $$PWD/*.h $$PWD/../include/
#first.depends = $(first) copydata
#export(first.depends)
#export(copydata.commands)
#QMAKE_EXTRA_TARGETS += first copydata

#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

#headers.path    = $$PWD/../include
#headers.files   += $$HEADERS
#INSTALLS       += headers


INCLUDEPATH += $$(MACE_ROOT)/include
