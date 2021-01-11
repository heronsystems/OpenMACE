#-------------------------------------------------
#
# Project created by QtCreator 2017-11-02T19:48:03
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = base_topic
TEMPLATE = lib

DEFINES += BASE_TOPIC_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    measurements/topic_trackangle.cpp \
    pose/topic_geodetic_position.cpp \
    pose/topic_cartesian_position.cpp \
    pose/topic_cartesian_velocity.cpp \
    pose/topic_altitude.cpp \
    pose/topic_agent_orientation.cpp \
    measurements/topic_airspeed.cpp \
    measurements/topic_groundspeed.cpp \
    pose/topic_rotational_velocity.cpp \
    vehicle/vehicle_path_linear_topic.cpp

HEADERS += \
    base_topic_global.h \
    base_topic_components.h \
    measurements/topic_trackangle.h \
    pose/topic_geodetic_position.h \
    pose/topic_cartesian_position.h \
    pose/topic_cartesian_velocity.h \
    pose/topic_altitude.h \
    pose/topic_agent_orientation.h \
    measurements/topic_speed.h \
    pose/topic_rotational_velocity.h \
    vehicle/vehicle_path_linear_topic.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/base_topic.lib release/base_topic.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/base_topic.lib debug/base_topic.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/


contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("base_topic: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("base_topic: Using standard mavlink libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase
