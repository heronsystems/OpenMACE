#-------------------------------------------------
#
# Project created by QtCreator 2018-09-07T13:26:04
#
#-------------------------------------------------

QT       -= core gui

TARGET = module_external_link_offset
TEMPLATE = lib

DEFINES += MODULE_EXTERNAL_LINK_OFFSET_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        module_external_link_offset.cpp

HEADERS += \
        module_external_link_offset.h \
        module_external_link_offset_global.h 

unix {
    target.path = /usr/lib
    INSTALLS += target
}
