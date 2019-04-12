QT += core
QT -= gui

CONFIG += c++11

TARGET = serverTest
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/release/ -lmodule_ground_station
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/debug/ -lmodule_ground_station
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_ground_station/ -lmodule_ground_station

INCLUDEPATH += $$PWD/../module_ground_station
DEPENDPATH += $$PWD/../module_ground_station


INCLUDEPATH += $$(MACE_ROOT)/include

unix{
    EigenInclude = $$system(pkg-config --cflags eigen3)
    EigenInclude = $$replace(EigenInclude, "-I", "")/eigen3
    INCLUDEPATH += $$EigenInclude
}
win32{
    INCLUDEPATH += "C:\Program Files (x86)\Eigen\include\eigen3"
}
