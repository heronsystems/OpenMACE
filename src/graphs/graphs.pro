QT      += core
QT      -= gui

TARGET = graphs
TEMPLATE = lib
DEFINES += GRAPHS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    signed_distance_fields/collision_map.cpp \
    signed_distance_fields/dynamic_spatial_hashed_collision_map.cpp \
    signed_distance_fields/sdf.cpp \
    signed_distance_fields/sdf_builder.cpp \
    signed_distance_fields/tagged_object_collision_map.cpp

HEADERS += \
    graphs_global.h \
    signed_distance_fields/arc_utilities/arc_helpers.hpp \
    signed_distance_fields/arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp \
    signed_distance_fields/arc_utilities/eigen_helpers.hpp \
    signed_distance_fields/arc_utilities/eigen_helpers_conversions.hpp \
    signed_distance_fields/arc_utilities/pretty_print.hpp \
    signed_distance_fields/arc_utilities/voxel_grid.hpp \
    signed_distance_fields/arc_utilities/zlib_helpers.hpp \
    signed_distance_fields/collision_map.hpp \
    signed_distance_fields/dynamic_spatial_hashed_collision_map.hpp \
    signed_distance_fields/sdf.hpp \
    signed_distance_fields/sdf_builder.hpp \
    signed_distance_fields/sdf_generation.hpp \
    signed_distance_fields/tagged_object_collision_map.hpp \
    signed_distance_fields/topology_computation.hpp


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
win32:CONFIG(release, debug|release):       lib.files   += release/graphs.lib release/graphs.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/graphs.lib debug/graphs.dll
INSTALLS += lib


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../spdlog/
INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

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

