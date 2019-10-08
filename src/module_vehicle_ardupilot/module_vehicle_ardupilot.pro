#-------------------------------------------------
#
# Project created by QtCreator 2017-01-12T14:51:44
#
#-------------------------------------------------

QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = module_vehicle_ardupilot
TEMPLATE = lib

DEFINES += MODULE_VEHICLE_ARDUPILOT_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += module_vehicle_ardupilot.cpp \
    ardupilot_states/state_grounded_disarmed.cpp \
    module_vehicle_ardupilot_mission_parser.cpp \
    ardupilot_states/abstract_state_ardupilot.cpp \
    ardupilot_states/state_takeoff_climbing.cpp \
    ardupilot_states/state_takeoff_transitioning.cpp \
    ardupilot_states/state_flight_guided.cpp \
    ardupilot_states/state_grounded_idle.cpp \
    ardupilot_states/state_grounded_arming.cpp \
    ardupilot_states/state_grounded_disarming.cpp \
    ardupilot_states/state_grounded_armed.cpp \
    ardupilot_states/state_grounded.cpp \
    ardupilot_states/state_takeoff.cpp \
    ardupilot_states/state_landing_transitioning.cpp \
    ardupilot_states/state_landing_descent.cpp \
    ardupilot_states/state_landing.cpp \
    ardupilot_states/state_flight_manual.cpp \
    ardupilot_states/state_flight_auto.cpp \
    ardupilot_states/state_flight_brake.cpp \
    vehicle_object/ardupilot_vehicle_object.cpp \
    vehicle_object/ardupilot_component_flight_mode.cpp \
    ardupilot_target_progess.cpp \
    ardupilot_states/state_takeoff_complete.cpp \
    ardupilot_states/state_flight.cpp \
    ardupilot_states/state_landing_complete.cpp \
    ardupilot_states/state_flight_rtl.cpp \
    ardupilot_states/state_flight_unknown.cpp \
    ardupilot_states/state_flight_land.cpp \
    ardupilot_states/state_flight_loiter.cpp \
    ardupilot_states/state_unknown.cpp \
    ardupilot_states/abstract_root_state.cpp \
    ardupilot_states/state_flight_guided_queue.cpp \
    ardupilot_states/state_flight_guided_idle.cpp \
    guided_timeout_controller.cpp \
    ardupilot_states/state_flight_guided_target_geo.cpp \
    ardupilot_states/state_flight_guided_target_car.cpp \
    ardupilot_states/state_flight_guided_spatial_item.cpp \
    ardupilot_states/state_flight_guided_target_att.cpp

HEADERS += module_vehicle_ardupilot.h\
    ardupilot_states/state_grounded_disarmed.h \
        module_vehicle_ardupilot_global.h \
    ardupilot_states/abstract_state_ardupilot.h \
    ardupilot_states/ardupilot_hsm.h \
    ardupilot_states/ardupilot_state_types.h \
    ardupilot_states/state_components.h \
    ardupilot_states/state_flight_guided.h \
    ardupilot_states/state_grounded.h \
    ardupilot_states/state_grounded_armed.h \
    ardupilot_states/state_grounded_arming.h \
    ardupilot_states/state_grounded_disarming.h \
    ardupilot_states/state_grounded_idle.h \
    ardupilot_states/state_takeoff_climbing.h \
    ardupilot_states/state_takeoff_transitioning.h \
    ardupilot_states/state_takeoff.h \
    ardupilot_states/state_landing_transitioning.h \
    ardupilot_states/state_landing_descent.h \
    ardupilot_states/state_landing.h \
    ardupilot_states/state_flight_manual.h \
    ardupilot_states/state_flight_auto.h \
    ardupilot_states/state_flight_brake.h \
    vehicle_object/ardupilot_vehicle_object.h \
    vehicle_object/ardupilot_component_flight_mode.h \
    ardupilot_target_progess.h \
    ardupilot_states/state_takeoff_complete.h \
    ardupilot_states/state_flight.h \
    ardupilot_states/state_landing_complete.h \
    ardupilot_states/state_flight_rtl.h \
    ardupilot_states/state_flight_unknown.h \
    ardupilot_states/state_flight_land.h \
    ardupilot_states/state_flight_loiter.h \
    ardupilot_states/state_unknown.h \
    ardupilot_states/abstract_root_state.h \
    ardupilot_states/state_flight_guided_queue.h \
    ardupilot_states/state_flight_guided_idle.h \
    guided_timeout_controller.h \
    ardupilot_states/state_flight_guided_target_geo.h \
    ardupilot_states/state_flight_guided_target_car.h \
    ardupilot_states/state_flight_guided_spatial_item.h \
    ardupilot_states/state_flight_guided_target_att.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_vehicle_ardupilot.lib release/module_vehicle_ardupilot.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_vehicle_ardupilot.lib debug/module_vehicle_ardupilot.dll
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
    exists(/opt/ros/kinetic/lib/) {
        DEFINES += ROS_EXISTS
        INCLUDEPATH += /opt/ros/kinetic/include
        INCLUDEPATH += /opt/ros/kinetic/lib
        LIBS += -L/opt/ros/kinetic/lib -loctomath
        LIBS += -L/opt/ros/kinetic/lib -loctomap
    } else {
        INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
        LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath
    }
}
win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
win32:INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
