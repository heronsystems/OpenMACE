#-------------------------------------------------
#
# Project created by QtCreator 2017-02-06T09:29:35
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = data_generic_mission_item_topic
TEMPLATE = lib

DEFINES += DATA_GENERIC_MISSION_ITEM_TOPIC_LIBRARY

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
    mission_item_topic.cpp \
    mission_list_topic.cpp \
    mission_item_request_topic.cpp \
    mission_home_topic.cpp \
    mission_item_reached_topic.cpp \
    mission_item_current_topic.cpp \
    vehicle_target_topic.cpp

HEADERS +=\
        data_generic_mission_item_topic_global.h \
    mission_item_topic.h \
    mission_item_topic_components.h \
    mission_list_topic.h \
    mission_item_request_topic.h \
    mission_home_topic.h \
    mission_item_reached_topic.h \
    mission_item_current_topic.h \
    vehicle_target_topic.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_generic_mission_item_topic.lib release/data_generic_mission_item_topic.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_generic_mission_item_topic.lib debug/data_generic_mission_item_topic.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix:!macx: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

