#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T14:51:44
#
#-------------------------------------------------

QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = module_vehicle_arduplane
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_ARDUPLANE_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += module_vehicle_arduplane.cpp \
    plane_flight_states/state_flight_AI.cpp \
    plane_flight_states/state_flight_AI_abort.cpp \
    plane_flight_states/state_flight_AI_execute.cpp \
    plane_flight_states/state_flight_AI_execute_abort.cpp \
    plane_flight_states/state_flight_AI_execute_deflection.cpp \
    plane_flight_states/state_flight_AI_execute_end.cpp \
    plane_flight_states/state_flight_AI_initialize.cpp \
    plane_flight_states/state_flight_AI_initialize_ABORT.cpp \
    plane_flight_states/state_flight_AI_initialize_ROUTE.cpp \
    vehicle_object/arduplane_component_flight_mode.cpp \
    vehicle_object/vehicle_object_arduplane.cpp \
    plane_flight_states/state_grounded_disarmed.cpp \
    plane_flight_states/state_takeoff_climbing.cpp \
    plane_flight_states/state_takeoff_transitioning.cpp \
    plane_flight_states/state_flight_guided.cpp \
    plane_flight_states/state_grounded_idle.cpp \
    plane_flight_states/state_grounded_arming.cpp \
    plane_flight_states/state_grounded_disarming.cpp \
    plane_flight_states/state_grounded_armed.cpp \
    plane_flight_states/state_grounded.cpp \
    plane_flight_states/state_takeoff.cpp \
    plane_flight_states/state_landing_transitioning.cpp \
    plane_flight_states/state_landing_descent.cpp \
    plane_flight_states/state_landing.cpp \
    plane_flight_states/state_flight_manual.cpp \
    plane_flight_states/state_flight_auto.cpp \
    plane_flight_states/state_takeoff_complete.cpp \
    plane_flight_states/state_flight.cpp \
    plane_flight_states/state_landing_complete.cpp \
    plane_flight_states/state_flight_rtl.cpp \
    plane_flight_states/state_flight_unknown.cpp \
    plane_flight_states/state_flight_land.cpp \
    plane_flight_states/state_flight_loiter.cpp \
    plane_flight_states/state_unknown.cpp \
    plane_flight_states/state_flight_guided_queue.cpp \
    plane_flight_states/state_flight_guided_idle.cpp \
    plane_flight_states/state_flight_guided_target_geo.cpp \
    plane_flight_states/state_flight_guided_target_car.cpp \
    plane_flight_states/state_flight_guided_spatial_item.cpp \
    plane_flight_states/state_flight_guided_target_att.cpp

HEADERS += module_vehicle_arduplane.h\
    module_vehicle_arduplane_global.h \
    plane_flight_states/arduplane_state_components.h \
    plane_flight_states/state_flight_AI.h \
    plane_flight_states/state_flight_AI_abort.h \
    plane_flight_states/state_flight_AI_execute.h \
    plane_flight_states/state_flight_AI_execute_abort.h \
    plane_flight_states/state_flight_AI_execute_deflection.h \
    plane_flight_states/state_flight_AI_execute_end.h \
    plane_flight_states/state_flight_AI_initialize.h \
    plane_flight_states/state_flight_AI_initialize_ABORT.h \
    plane_flight_states/state_flight_AI_initialize_ROUTE.h \
    vehicle_object/arduplane_component_flight_mode.h \
    vehicle_object/vehicle_object_arduplane.h \
    plane_flight_states/state_grounded_disarmed.h \
    plane_flight_states/state_flight_guided.h \
    plane_flight_states/state_grounded.h \
    plane_flight_states/state_grounded_armed.h \
    plane_flight_states/state_grounded_arming.h \
    plane_flight_states/state_grounded_disarming.h \
    plane_flight_states/state_grounded_idle.h \
    plane_flight_states/state_takeoff_climbing.h \
    plane_flight_states/state_takeoff_transitioning.h \
    plane_flight_states/state_takeoff.h \
    plane_flight_states/state_landing_transitioning.h \
    plane_flight_states/state_landing_descent.h \
    plane_flight_states/state_landing.h \
    plane_flight_states/state_flight_manual.h \
    plane_flight_states/state_flight_auto.h \
    plane_flight_states/state_takeoff_complete.h \
    plane_flight_states/state_flight.h \
    plane_flight_states/state_landing_complete.h \
    plane_flight_states/state_flight_rtl.h \
    plane_flight_states/state_flight_unknown.h \
    plane_flight_states/state_flight_land.h \
    plane_flight_states/state_flight_loiter.h \
    plane_flight_states/state_unknown.h \
    plane_flight_states/state_flight_guided_queue.h \
    plane_flight_states/state_flight_guided_idle.h \
    plane_flight_states/state_flight_guided_target_geo.h \
    plane_flight_states/state_flight_guided_target_car.h \
    plane_flight_states/state_flight_guided_spatial_item.h \
    plane_flight_states/state_flight_guided_target_att.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3


contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("module_vehicle_arduplane: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("module_vehicle_arduplane: Using standard ardupilot libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_arduplane.lib release/module_vehicle_arduplane.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_arduplane.lib debug/module_vehicle_arduplane.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/release/ -lcommsMAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/debug/ -lcommsMAVLINK
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMAVLINK/ -lcommsMAVLINK

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/release/ -lmodule_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/debug/ -lmodule_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/ -lmodule_vehicle_ardupilot

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/release/ -ldata_interface_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/debug/ -ldata_interface_MAVLINK
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/ -ldata_interface_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../trajectory_control/release/ -ltrajectory_control
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../trajectory_control/debug/ -ltrajectory_control
else:unix:!macx: LIBS += -L$$OUT_PWD/../trajectory_control/ -ltrajectory_control


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
