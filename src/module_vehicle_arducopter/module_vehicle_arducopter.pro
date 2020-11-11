#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T14:51:44
#
#-------------------------------------------------

QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = module_vehicle_arducopter
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_ARDUCOPTER_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += module_vehicle_arducopter.cpp \
    flight_states/state_grounded_disarmed.cpp \
    flight_states/state_takeoff_climbing.cpp \
    flight_states/state_takeoff_transitioning.cpp \
    flight_states/state_flight_guided.cpp \
    flight_states/state_grounded_idle.cpp \
    flight_states/state_grounded_arming.cpp \
    flight_states/state_grounded_disarming.cpp \
    flight_states/state_grounded_armed.cpp \
    flight_states/state_grounded.cpp \
    flight_states/state_takeoff.cpp \
    flight_states/state_landing_transitioning.cpp \
    flight_states/state_landing_descent.cpp \
    flight_states/state_landing.cpp \
    flight_states/state_flight_manual.cpp \
    flight_states/state_flight_auto.cpp \
    flight_states/state_flight_brake.cpp \
    flight_states/state_takeoff_complete.cpp \
    flight_states/state_flight.cpp \
    flight_states/state_landing_complete.cpp \
    flight_states/state_flight_rtl.cpp \
    flight_states/state_flight_unknown.cpp \
    flight_states/state_flight_land.cpp \
    flight_states/state_flight_loiter.cpp \
    flight_states/state_unknown.cpp \
    flight_states/state_flight_guided_queue.cpp \
    flight_states/state_flight_guided_idle.cpp \
    flight_states/state_flight_guided_target_geo.cpp \
    flight_states/state_flight_guided_target_car.cpp \
    flight_states/state_flight_guided_spatial_item.cpp \
    flight_states/state_flight_guided_target_att.cpp \
    vehicle_object/arducopter_component_flight_mode.cpp \
    vehicle_object/vehicle_object_arducopter.cpp

HEADERS += module_vehicle_arducopter.h\
    flight_states/state_grounded_disarmed.h \
        module_vehicle_arducopter_global.h \
    flight_states/arducopter_state_components.h \
    flight_states/state_flight_guided.h \
    flight_states/state_grounded.h \
    flight_states/state_grounded_armed.h \
    flight_states/state_grounded_arming.h \
    flight_states/state_grounded_disarming.h \
    flight_states/state_grounded_idle.h \
    flight_states/state_takeoff_climbing.h \
    flight_states/state_takeoff_transitioning.h \
    flight_states/state_takeoff.h \
    flight_states/state_landing_transitioning.h \
    flight_states/state_landing_descent.h \
    flight_states/state_landing.h \
    flight_states/state_flight_manual.h \
    flight_states/state_flight_auto.h \
    flight_states/state_flight_brake.h \
    flight_states/state_takeoff_complete.h \
    flight_states/state_flight.h \
    flight_states/state_landing_complete.h \
    flight_states/state_flight_rtl.h \
    flight_states/state_flight_unknown.h \
    flight_states/state_flight_land.h \
    flight_states/state_flight_loiter.h \
    flight_states/state_unknown.h \
    flight_states/state_flight_guided_queue.h \
    flight_states/state_flight_guided_idle.h \
    flight_states/state_flight_guided_target_geo.h \
    flight_states/state_flight_guided_target_car.h \
    flight_states/state_flight_guided_spatial_item.h \
    flight_states/state_flight_guided_target_att.h \
    vehicle_object/arducopter_component_flight_mode.h \
    vehicle_object/vehicle_object_arducopter.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../spdlog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_arducopter.lib release/module_vehicle_arducopter.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_arducopter.lib debug/module_vehicle_arducopter.dll
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

INCLUDEPATH += $$PWD/../data_interface_MAVLINK
DEPENDPATH += $$PWD/../data_interface_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

INCLUDEPATH += $$PWD/../base_topic
DEPENDPATH += $$PWD/../base_topic

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
