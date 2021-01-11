#-------------------------------------------------
#
# Project created by QtCreator 2016-08-31T13:55:42
#
#-------------------------------------------------
QT += serialport
QT += network
QT      += core
QT      -= gui

TARGET = comms
TEMPLATE = lib

DEFINES += COMMS_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += \
    serial_link.cpp \
    serial_configuration.cpp \
    mavlink_configuration.cpp \
    comms_marshaler.cpp \
    protocol_mavlink.cpp \
    udp_configuration.cpp \
    udp_link.cpp \
    tcp_link.cpp \
    tcp_configuration.cpp

HEADERS +=\
    i_link.h \
    serial_link.h \
    i_protocol.h \
    comms_global.h \
    i_link_events.h \
    serial_configuration.h \
    link_configuration.h \
    protocol_configuration.h \
    mavlink_configuration.h \
    comms_marshaler.h \
    i_protocol_events.h \
    i_protocol_mavlink_events.h \
    protocol_mavlink.h \
    comms_events.h \
    udp_configuration.h \
    udp_link.h \
    tcp_link.h \
    tcp_configuration.h

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("base: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("base: Using standard mavlink libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/comms.lib release/comms.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/comms.lib debug/comms.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

