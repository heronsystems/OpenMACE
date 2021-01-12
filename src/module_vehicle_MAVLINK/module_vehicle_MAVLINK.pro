#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:14:11
#
#-------------------------------------------------

QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = module_vehicle_MAVLINK
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_MAVLINK_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += \
    controllers/commands/command_msg_interval.cpp \
    controllers/controller_parameter_request.cpp \
    controllers/controller_set_surface_deflection.cpp \
    module_vehicle_mavlink.cpp \
    vehicle_object/mavlink_vehicle_object.cpp \
    vehicle_object/state_data_mavlink.cpp \
    vehicle_object/parse_mavlink.cpp \
    vehicle_object/status_data_mavlink.cpp \
    environment_object/environment_data_mavlink.cpp \
    controllers/controller_guided_mission_item.cpp \
    vehicle_object/mission_data_mavlink.cpp \
    controllers/controller_guided_target_item_local.cpp \
    controllers/controller_guided_target_item_global.cpp \
    controllers/controller_set_gps_global_origin.cpp \
    controllers/controller_guided_target_item_attitude.cpp

HEADERS += module_vehicle_mavlink.h\
    controllers/commands/command_change_speed.h \
    controllers/commands/command_set_surface_deflection.h \
    controllers/controller_guided_target_item_waypoint.h \
    controllers/controller_parameter_request.h \
    controllers/controller_set_surface_deflection.h \
    controllers/controller_vision_position_estimate.h \
    controllers/controller_write_event_to_log.h \
    module_vehicle_mavlink_global.h \
    controllers/controller_system_mode.h \
    controllers/commands/command_arm.h \
    controllers/commands/command_land.h \
    controllers/commands/command_rtl.h \
    controllers/commands/command_takeoff.h \
    controllers/commands/generic_long_command.h \
    vehicle_object/mavlink_vehicle_object.h \
    vehicle_object/state_data_mavlink.h \
    vehicle_object/status_data_mavlink.h \
    environment_object/environment_data_mavlink.h \
    controllers/controller_mission.h \
    controllers/controller_guided_mission_item.h \
    vehicle_object/mission_data_mavlink.h \
    controllers/commands/generic_int_command.h \
    mavlink_entity_key.h \
    controllers/controller_guided_target_item_local.h \
    controllers/controller_guided_target_item_global.h \
    controllers/common.h \
    controllers/commands/command_msg_interval.h \
    mavlink_coordinate_frames.h \
    controllers/controller_set_gps_global_origin.h \
    controllers/controller_guided_target_item_attitude.h \
    controllers/commands/command_msg_request.h \
    controllers/commands/command_home_position.h


INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("module_vehicle_mavlink: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("module_vehicle_mavlink: Using standard ardupilot libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_MAVLINK.lib release/module_vehicle_MAVLINK.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_MAVLINK.lib debug/module_vehicle_MAVLINK.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


#Header file copy
headers_controllers.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK/controllers
headers_controllers.files   += \
    controllers/controller_system_mode.h
INSTALLS       += headers_controllers

#Header file copy
headers_controller_commands.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK/controllers/commands
headers_controller_commands.files   += \
    controllers/commands/command_arm.h \
    controllers/commands/command_land.h \
    controllers/commands/command_rtl.h \
    controllers/commands/command_takeoff.h \
    controllers/commands/generic_long_command.h
INSTALLS       += headers_controller_commands

#Header file copy
headers_vehicle_object.path    = $$(MACE_ROOT)/include/module_vehicle_MAVLINK/vehicle_object
headers_vehicle_object.files   += \
    vehicle_object/mavlink_vehicle_object.h \
    vehicle_object/state_data_mavlink.h
INSTALLS       += headers_vehicle_object

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/release/ -lcommsMAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/debug/ -lcommsMAVLINK
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMAVLINK/ -lcommsMAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../controllers/release/ -lcontrollers
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../controllers/debug/ -lcontrollers
else:unix:!macx: LIBS += -L$$OUT_PWD/../controllers/ -lcontrollers

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/release/ -ldata_generic_command_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/debug/ -ldata_generic_command_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/ -ldata_generic_command_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/release/ -lmodule_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/debug/ -lmodule_vehicle_generic
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_generic/ -lmodule_vehicle_generic

unix {
    exists($$(ROS_ROOT_DIR)/lib/) {

      DEFINES += ROS_EXISTS
      INCLUDEPATH += $$(ROS_ROOT_DIR)/include
      INCLUDEPATH += $$(ROS_ROOT_DIR)/lib
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomath
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomap

    } else {
      message("ROS root" path has not been detected...)
      INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
      LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath

      # Octomap Warning suppression:
      QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include
    }
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
win32:INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
