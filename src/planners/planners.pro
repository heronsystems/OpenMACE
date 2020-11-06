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
    virtual_potential_fields/virtual_force.cpp \
    fast_marching/console/console.cpp \
    fast_marching/ndgridmap/cell.cpp \
    fast_marching/ndgridmap/fmcell.cpp \
    fast_marching/thirdparty/reference/queue.cpp

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
    virtual_potential_fields/virtual_force.h \
    fast_marching/console/console.h \
    fast_marching/datastructures/fmcompare.hpp \
    fast_marching/datastructures/fmdaryheap.hpp \
    fast_marching/datastructures/fmfibheap.hpp \
    fast_marching/datastructures/fmpriorityqueue.hpp \
    fast_marching/datastructures/fmuntidyqueue.hpp \
    fast_marching/fm/ddqm.hpp \
    fast_marching/fm/eikonalsolver.hpp \
    fast_marching/fm/fim.hpp \
    fast_marching/fm/fmm.hpp \
    fast_marching/fm/fmmstar.hpp \
    fast_marching/fm/fsm.hpp \
    fast_marching/fm/gmm.hpp \
    fast_marching/fm/lsm.hpp \
    fast_marching/fm/sfmm.hpp \
    fast_marching/fm/sfmmstar.hpp \
    fast_marching/fm/solver.hpp \
    fast_marching/fm/ufmm.hpp \
    fast_marching/fm2/fm2.hpp \
    fast_marching/fm2/fm2star.hpp \
    fast_marching/gradientdescent/gradientdescent.hpp \
    fast_marching/io/gridplotter.hpp \
    fast_marching/io/gridpoints.hpp \
    fast_marching/io/gridwriter.hpp \
    fast_marching/io/maploader.hpp \
    fast_marching/ndgridmap/cell.h \
    fast_marching/ndgridmap/fmcell.h \
    fast_marching/ndgridmap/ndgridmap.hpp \
    fast_marching/thirdparty/reference/queue.hpp \
    fast_marching/thirdparty/untidy_queue.hpp \
    fast_marching/utils/utils.h \
    fast_marching/io/CImg.h


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
INCLUDEPATH += $$PWD/../../spdlog/
INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3
# Octomap Warning suppression:
QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include3

unix {
INCLUDEPATH += /usr/include/boost
LIBS += -lm
LIBS += -lpthread
LIBS += -llz4
LIBS += -lX11
LIBS += -lz
}

win32 {
INCLUDEPATH += $$(MACE_ROOT)/tools/boost_local
LIBS += -L $$(MACE_ROOT)/tools/boost_local
LIBS += -lgdi32
LIBS += -lz
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

INCLUDEPATH += $$PWD/../maps
DEPENDPATH += $$PWD/../maps

unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann
unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann_s

INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
DEPENDPATH += $$PWD/../../tools/flann/src/cpp

# Flann Warning suppression:
QMAKE_CXXFLAGS += -isystem $$PWD/../../tools/flann/src/cpp

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:unix:!macx: LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath
