#-------------------------------------------------
#
# Project created by QtCreator 2017-05-17T16:35:57
#
#-------------------------------------------------


QT      += core
QT      -= gui

TARGET = data_generic_command_item
TEMPLATE = lib

DEFINES += DATA_GENERIC_COMMAND_ITEM_LIBRARY

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
    do_items/action_arm.cpp \
    do_items/action_change_mode.cpp \
    do_items/action_change_speed.cpp \
    do_items/action_motor_test.cpp \
    do_items/action_mission_command.cpp \
    do_items/action_set_surface_deflection.cpp \
    mace/ai_items/action_event_tag.cpp \
    mace/ai_items/action_initialize_test_setup.cpp \
    mace/ai_items/action_procedural_command.cpp \
    spatial_items/spatial_home.cpp \
    spatial_items/spatial_land.cpp \
    spatial_items/spatial_loiter_time.cpp \
    spatial_items/spatial_loiter_turns.cpp \
    spatial_items/spatial_loiter_unlimited.cpp \
    spatial_items/spatial_rtl.cpp \
    spatial_items/spatial_takeoff.cpp \
    spatial_items/spatial_waypoint.cpp \
    mission_items/mission_ack.cpp \
    mission_items/mission_list.cpp \
    command_item_ack.cpp \
    mission_items/mission_item_achieved.cpp \
    mission_items/mission_item_current.cpp \
    mission_items/mission_key.cpp \
    mission_items/mission_key_change.cpp \
    boundary_items/boundary_key.cpp \
    boundary_items/boundary_list.cpp \
    target_items/dynamic_target_kinematic.cpp \
    target_items/dynamic_target_list.cpp \
    target_items/dynamic_mission_queue.cpp \
    target_items/dynamic_target_state.cpp \
    spatial_items/abstract_spatial_action.cpp \
    mission_items/abstract_mission_item.cpp \
    interface_command_helper.cpp \
    mission_items/mission_item_factory.cpp \
    do_items/action_dynamic_target.cpp \
    do_items/action_message_interval.cpp \
    do_items/action_execute_spatial_item.cpp \
    do_items/action_set_global_origin.cpp \
    target_items/dynamic_target_orientation.cpp \
    target_items/abstract_dynamic_target.cpp \
    do_items/action_message_request.cpp

HEADERS +=\
    boundary_items/environment_boundary.h \
    do_items/action_arm.h \
    do_items/action_change_mode.h \
    do_items/action_change_speed.h \
    do_items/action_motor_test.h\
    do_items/action_set_surface_deflection.h \
    do_items/do_components.h \
    do_items/action_mission_command.h \
    mace/ai_items/abstract_ai_command.h \
    mace/ai_items/action_event_tag.h \
    mace/ai_items/action_initialize_test_setup.h \
    mace/ai_items/action_procedural_command.h \
    mace/ai_items/ai_command_components.h \
    mace/ai_items/ai_command_type.h \
    mission_items/mission_ack.h \
    mission_items/mission_list.h \
    spatial_items/spatial_components.h \
    spatial_items/spatial_home.h \
    spatial_items/spatial_land.h \
    spatial_items/spatial_loiter_time.h \
    spatial_items/spatial_loiter_turns.h \
    spatial_items/spatial_loiter_unlimited.h \
    spatial_items/spatial_rtl.h \
    spatial_items/spatial_takeoff.h \
    spatial_items/spatial_waypoint.h \
    data_generic_command_item_global.h \
    command_item_components.h \
    abstract_command_item.h \
    command_item_ack.h \
    mission_items/mission_item_achieved.h \
    mission_items/mission_item_current.h \
    mission_items/mission_state.h \
    mission_items/mission_key.h \
    mission_items/mission_key_change.h \
    command_item_type.h \
    boundary_items/boundary_key.h \
    boundary_items/boundary_type.h \
    boundary_items/boundary_list.h \
    target_items/dynamic_target_kinematic.h \
    target_items/dynamic_target_list.h \
    target_items/dynamic_mission_queue.h \
    spatial_items/abstract_spatial_action.h \
    mission_items/typedef_mission_types.h \
    target_items/dynamic_target_state.h \
    mission_items/mission_item_interface.h \
    mission_items/abstract_mission_item.h \
    interface_command_helper.h \
    mission_items/mission_item_factory.h \
    do_items/action_dynamic_target.h \
    do_items/action_message_interval.h \
    do_items/action_execute_spatial_item.h \
    do_items/action_set_global_origin.h \
    target_items/dynamic_target_orientation.h \
    target_items/abstract_dynamic_target.h \
    do_items/action_message_request.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_generic_command_item.lib release/data_generic_command_item.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_generic_command_item.lib debug/data_generic_command_item.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("data_generic_command_item: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("data_generic_command_item: Using standard ardupilot libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

# Eigen Warning suppression:
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

