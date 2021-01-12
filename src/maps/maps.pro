#-------------------------------------------------
#
# Project created by QtCreator 2017-08-17T19:32:56
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = maps
TEMPLATE = lib

DEFINES += MAPS_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
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

SOURCES += \
    bounded_2D_grid.cpp \
    base_grid_map.cpp \
    iterators/grid_map_iterator.cpp \
    iterators/polygon_map_iterator.cpp \
    iterators/circle_map_iterator.cpp \
    iterators/generic_map_iterator.cpp \
    octomap_wrapper.cpp \
    occupancy_2d_grid_topic.cpp \
    occupancy_map_2D_inflated.cpp \
    layered_map.cpp \
    map_cell.cpp

HEADERS +=\
        maps_global.h \
    dynamic_2D_grid.h \
    bounded_2D_grid.h \
    base_grid_map.h \
    data_2d_grid.h \
    iterators/polygon_map_iterator.h \
    iterators/circle_map_iterator.h \
    iterators/grid_map_iterator.h \
    iterators/generic_map_iterator.h \
    dynamic_2D_grid.tpp \
    octomap_wrapper.h \
    octomap_sensor_definition.h \
    occupancy_2d_grid_topic.h \
    map_topic_components.h \
    octomap_2d_projection_definition.h \
    occupancy_map_2D_inflated.h \
    occupancy_definition.h \
    layered_map.h \
    map_cell.h \
    tests/maps_tests.h

#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/maps.lib release/maps.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/maps.lib debug/maps.dll
INSTALLS += lib

#Necessary includes
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("maps: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("maps: Using standard mavlink libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
# Octomap Warning suppression:
QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:unix:!macx: LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath
