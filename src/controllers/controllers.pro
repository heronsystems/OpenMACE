#-------------------------------------------------
#
# Project created by QtCreator 2018-02-20T12:27:44
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = controllers
TEMPLATE = lib

DEFINES += MACE_CONTROLLERS_LIBRARY

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

SOURCES +=

HEADERS += \
    controllers_global.h \
    actions/action_base.h \
    actions/action_broadcast.h \
    actions/action_final_receive_respond.h \
    actions/action_finish.h \
    actions/action_intermediate_receive.h \
    actions/action_intermediate_respond.h \
    actions/action_intermediate.h \
    actions/action_request.h \
    actions/action_send.h \
    controllers_global.h \
    I_controller.h \
    generic_controller.h \
    I_message_notifier.h \
    base_data_item.h \
    actions/action_unsolicited_receive.h \
    controller_collection.h \
    actions/action_intermediate_unsolicited.h \
    actions/action_intermediate_unsolicited_receive.h \
    actions/action_unsolicited_receive_respond.h \
    actions/action_broadcast_reliable.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("controllers: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("controllers: Using standard mavlink libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/controllers.lib release/controllers.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/controllers.lib debug/controllers.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

SUBDIRS += \
    controllers.pro

DISTFILES += \
    actions/readme.md
