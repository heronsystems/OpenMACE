#-------------------------------------------------
#
# Project created by QtCreator 2017-09-16T11:10:32
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = planners
TEMPLATE = lib

DEFINES += PLANNERS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

QMAKE_CXXFLAGS_WARN_ON += -Wno-unknown-pragmas

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
        planners.cpp \
    nearest_neighbor_linear.cpp \
    tsp_greedy_nearest_neighbor.cpp \
    tsp_2opt.cpp \
    probabilistic_roadmap.cpp \
    rrt_base.cpp \
#    nearest_neighbor_flann.cpp
    path_reduction.cpp \
    graph_planning_node.cpp \
    a_star_base.cpp \
    virtual_potential_fields/potential_fields.cpp \
    virtual_potential_fields/virtual_force.cpp

HEADERS += \
        planners.h \
        planners_global.h \ 
    nearest_neighbor_linear.h \
    tsp_greedy_nearest_neighbor.h \
    tsp_2opt.h \
    probabilistic_roadmap.h \
    rrt_base.h \
    nearest_neighbor_flann.h \
    nearest_neighbor_abstract.h \
    rrt_node.h \
    path_reduction.h \
    graph_planning_node.h \
    a_star_base.h \
    virtual_potential_fields/potential_fields.h \
    virtual_potential_fields/virtual_force.h


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
win32:CONFIG(release, debug|release):       lib.files   += release/planners.lib release/planners.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/planners.lib debug/planners.dll
INSTALLS += lib


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

INCLUDEPATH += $$PWD/../maps
DEPENDPATH += $$PWD/../maps

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../tools/flann/build/lib/release/ -lflann
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../tools/flann/build/lib/debug/ -lflann
#else:unix:!macx: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann

#INCLUDEPATH += $$PWD/../../tools/flann/build
#DEPENDPATH += $$PWD/../../tools/flann/build

unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann
unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann_s

INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
DEPENDPATH += $$PWD/../../tools/flann/src/cpp

unix: LIBS += -llz4
