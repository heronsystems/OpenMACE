#-------------------------------------------------
#
# Project created by QtCreator 2017-01-27T09:40:19
#
#-------------------------------------------------

QT       -= core gui

TARGET = data_vehicle_commands
TEMPLATE = lib

DEFINES += DATA_VEHICLE_COMMANDS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11

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
    actionCommandComponents/command_vehicle_mode.cpp \
    actionCommandComponents/command_vehicle_takeoff.cpp \
    actionCommandComponents/command_vehicle_land.cpp \
    action_command_topic.cpp \
    mission_command_topic.cpp \
    missionCommandComponents/command_mission_waypoint.cpp \
    actionCommandComponents/command_vehicle_arm.cpp \
    actionCommandComponents/command_vehicle_rtl.cpp \
    vehicle_mission_list.cpp

HEADERS +=\
        data_vehicle_commands_global.h \
    command_types.h \
    actionCommandComponents/command_vehicle_mode.h \
    actionCommandComponents/command_vehicle_takeoff.h \
    actionCommandComponents/command_vehicle_land.h \
    mission_components.h \
    action_components.h \
    abstract_action_command.h \
    action_command_topic.h \
    mission_command_topic.h \
    missionCommandComponents/command_mission_waypoint.h \
    actionCommandComponents/command_vehicle_arm.h \
    actionCommandComponents/command_vehicle_rtl.h \
    vehicle_mission_list.h \
    abstract_mission_item.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_vehicle_commands.lib release/data_vehicle_commands.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_vehicle_commands.lib debug/data_vehicle_commands.dll
INSTALLS += lib

#Header file copy
headers.path    = $$(MACE_ROOT)/include/data_vehicle_commands
headers.files   += \
    data_vehicle_commands_global.h \
    command_types.h \
    mission_components.h \
    action_components.h \
    abstract_action_command.h \
    abstract_mission_item.h \
    action_command_topic.h \
    mission_command_topic.h \
    vehicle_mission_list.h
INSTALLS       += headers

#Header file copy
headers_actionComponents.path    = $$(MACE_ROOT)/include/data_vehicle_commands/actionCommandComponents
headers_actionComponents.files   += \
    actionCommandComponents/command_vehicle_mode.h \
    actionCommandComponents/command_vehicle_takeoff.h \
    actionCommandComponents/command_vehicle_land.h \
    actionCommandComponents/command_vehicle_arm.h \
    actionCommandComponents/command_vehicle_rtl.h

INSTALLS       += headers_actionComponents

#Header file copy
headers_missionComponents.path    = $$(MACE_ROOT)/include/data_vehicle_commands/missionCommandComponents
headers_missionComponents.files   += \
    missionCommandComponents/command_mission_waypoint.h \


INSTALLS       += headers_missionComponents

INCLUDEPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item
