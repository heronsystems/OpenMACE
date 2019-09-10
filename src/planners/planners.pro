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
    virtual_potential_fields/virtual_force.cpp \
    fast_marching/console/console.cpp \
    fast_marching/ndgridmap/cell.cpp \
    fast_marching/ndgridmap/fmcell.cpp \
    signed_distance_fields/collision_map.cpp \
    signed_distance_fields/compute_convex_segments_test.cpp \
    signed_distance_fields/dynamic_spatial_hashed_collision_map.cpp \
    signed_distance_fields/estimate_distance_test.cpp \
    signed_distance_fields/image_2d_sdf_node.cpp \
    signed_distance_fields/sdf.cpp \
    signed_distance_fields/sdf_builder.cpp \
    signed_distance_fields/sdf_generation_node.cpp \
    signed_distance_fields/sdf_tools_tutorial.cpp \
    signed_distance_fields/tagged_object_collision_map.cpp \
    signed_distance_fields/test_voxel_grid.cpp \
    signed_distance_fields/arc_utilities/base64_helpers.cpp \
    signed_distance_fields/arc_utilities/first_order_deformation.cpp \
    signed_distance_fields/arc_utilities/time_optimal_trajectory_parametrization.cpp \
    signed_distance_fields/arc_utilities/timing.cpp \
    signed_distance_fields/arc_utilities/zlib_helpers.cpp

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
    fast_marching/thirdparty/reference/fast_marching.hpp \
    fast_marching/thirdparty/reference/queue.hpp \
    fast_marching/thirdparty/untidy_queue.hpp \
    fast_marching/utils/utils.h \
    fast_marching/io/CImg.h \
    signed_distance_fields/collision_map.hpp \
    signed_distance_fields/dynamic_spatial_hashed_collision_map.hpp \
    signed_distance_fields/sdf.hpp \
    signed_distance_fields/sdf_builder.hpp \
    signed_distance_fields/sdf_generation.hpp \
    signed_distance_fields/tagged_object_collision_map.hpp \
    signed_distance_fields/topology_computation.hpp \
    signed_distance_fields/arc_utilities/aligned_eigen_types.hpp \
    signed_distance_fields/arc_utilities/arc_exceptions.hpp \
    signed_distance_fields/arc_utilities/arc_helpers.hpp \
    signed_distance_fields/arc_utilities/base64_helpers.hpp \
    signed_distance_fields/arc_utilities/dijkstras.hpp \
    signed_distance_fields/arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp \
    signed_distance_fields/arc_utilities/eigen_helpers.hpp \
    signed_distance_fields/arc_utilities/eigen_helpers_conversions.hpp \
    signed_distance_fields/arc_utilities/eigen_typedefs.hpp \
    signed_distance_fields/arc_utilities/filesystem.hpp \
    signed_distance_fields/arc_utilities/first_order_deformation.h \
    signed_distance_fields/arc_utilities/get_neighbours.hpp \
    signed_distance_fields/arc_utilities/hash.hpp \
    signed_distance_fields/arc_utilities/log.hpp \
    signed_distance_fields/arc_utilities/math_helpers.hpp \
    signed_distance_fields/arc_utilities/maybe.hpp \
    signed_distance_fields/arc_utilities/path_utils.hpp \
    signed_distance_fields/arc_utilities/pretty_print.hpp \
    signed_distance_fields/arc_utilities/ros_helpers.hpp \
    signed_distance_fields/arc_utilities/serialization.hpp \
    signed_distance_fields/arc_utilities/serialization_eigen.hpp \
    signed_distance_fields/arc_utilities/serialization_ros.hpp \
    signed_distance_fields/arc_utilities/simple_astar_planner.hpp \
    signed_distance_fields/arc_utilities/simple_dtw.hpp \
    signed_distance_fields/arc_utilities/simple_hausdorff_distance.hpp \
    signed_distance_fields/arc_utilities/simple_hierarchical_clustering.hpp \
    signed_distance_fields/arc_utilities/simple_kmeans_clustering.hpp \
    signed_distance_fields/arc_utilities/simple_prm_planner.hpp \
    signed_distance_fields/arc_utilities/simple_rrt_planner.hpp \
    signed_distance_fields/arc_utilities/thin_plate_spline.hpp \
    signed_distance_fields/arc_utilities/time_optimal_trajectory_parametrization.hpp \
    signed_distance_fields/arc_utilities/timing.hpp \
    signed_distance_fields/arc_utilities/vector_math.hpp \
    signed_distance_fields/arc_utilities/voxel_grid.hpp \
    signed_distance_fields/arc_utilities/zlib_helpers.hpp \
    signed_distance_fields/arc_utilities/portable_endian.h


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
INCLUDEPATH += $$PWD/../../tools/boost/boost_install/include
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
LIBS += -lz
