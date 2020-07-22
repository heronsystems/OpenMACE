#-------------------------------------------------
#
# Project created by QtCreator 2017-08-27T11:33:13
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = base
TEMPLATE = lib

DEFINES += BASE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#CONFIG += c++14 #This will build in C++14

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
    pose/twist.cpp \
    pose/velocity_interface_translational.cpp \
    state_space/real_vector.cpp \
    pose/cartesian_position_3D.cpp \
    pose/cartesian_position_2D.cpp \
    state_space/real_vector_bounds.cpp \
    pose/geodetic_position_2D.cpp \
    state_space/state_space.cpp \
    state_space/cartesian_2D_space.cpp \
    math/random_number_generator.cpp \
    math/cost.cpp \
    state_space/goal_state.cpp \
    state_space/space_information.cpp \
    state_space/motion_validity_check.cpp \
    state_space/discrete_motion_validity_check.cpp \
    state_space/special_validity_check.cpp \
    geometry/cell_2DC.cpp \
    state_space/start_state.cpp \
    pose/geodetic_position_3D.cpp \
    pose/dynamics_aid.cpp \
    geometry/polygon_2DG.cpp \
    pose/abstract_cartesian_position.cpp \
    pose/abstract_altitude.cpp \
    pose/abstract_geodetic_position.cpp \
    geometry/polygon_2DC.cpp \
    pose/base_altitude.cpp \
    pose/abstract_velocity.cpp \
    geometry/abstract_polygon.cpp \
    pose/rotation_3D.cpp \
    measurements/base_speed.cpp \
    pose/abstract_rotation.cpp \
    pose/abstract_position.cpp \
    pose/rotation_2D.cpp \
    misc/kinematic_definitions.cpp \
    pose/pose.cpp

HEADERS +=\
    base_global.h \
    math/helper_pi.h \
    pose/cartesian_position_2D.h \
    pose/cartesian_position_3D.h \
    geometry/base_polygon.h \
    pose/twist.h \
    pose/velocity_interface_rotational.h \
    pose/velocity_interface_translational.h \
    state_space/real_vector.h \
    geometry/geometry_helper.h \
    state_space/real_vector_bounds.h \
    pose/geodetic_position_2D.h \
    state_space/state_sampler.h \
    state_space/state_space_types.h \
    state_space/state_space.h \
    state_space/state.h \
    state_space/space_information.h \
    state_space/cartesian_2D_space.h \
    math/random_number_generator.h \
    math/cost.h \
    state_space/generic_goal.h \
    state_space/goal_state.h \
    state_space/abstract_motion_validity_check.h \
    state_space/abstract_state_validity_check.h \
    state_space/discrete_motion_validity_check.h \
    state_space/special_validity_check.h \
    geometry/base_line.h \
    geometry/base_line.h \
    geometry/base_polygon.h \
    geometry/cell_2DC.h \
    geometry/geometry_helper.h \
    geometry/base_line.h \
    pose/abstract_velocity.h \
    state_space/start_state.h \
    state_space/generic_start.h \
    pose/geodetic_position_3D.h \
    pose/dynamics_aid.h \
    geometry/polygon_2DG.h \
    geometry/rotate_2d.h \
    math/math_components.h \
    pose/abstract_cartesian_position.h \
    pose/position_interface.h \
    geometry/polygon_cartesian.h \
    pose/altitude_interface.h \
    pose/abstract_altitude.h \
    pose/abstract_geodetic_position.h \
    misc/local_coordindate_frames.h \
    misc/global_coordinate_frames.h \
    misc/altitude_coordinate_frames.h \
    misc/coordinate_frame_components.h \
    misc/kinematic_definitions.h \
    pose/base_altitude.h \
    geometry/abstract_polygon.h \
    unit_tests/unittests_orientation.h \
    pose/rotation_3D.h \
    measurements/base_speed.h \
    pose/abstract_rotation.h \
    pose/abstract_position.h \
    pose/rotation_2D.h \
    pose/pose_components.h \
    pose/velocity_helper.h \
    misc/dimension.h \
    pose/pose.h \
    unit_tests/unittests_position.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/base.lib release/base.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/base.lib debug/base.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

