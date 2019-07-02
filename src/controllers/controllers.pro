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

QMAKE_CXXFLAGS += -std=c++11

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
    generic_controller_queue_data_with_module.h \
    I_message_notifier.h \
    base_data_item.h \
    actions/action_unsolicited_receive.h \
    controller_collection.h \
    actions/action_intermediate_unsolicited.h \
    actions/action_intermediate_unsolicited_receive.h \
    actions/action_unsolicited_receive_respond.h \
    actions/action_broadcast_reliable.h


# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/controllers.lib release/controllers.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/controllers.lib debug/controllers.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)



INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3

#INCLUDEPATH += $$(MACE_DIGIMESH_WRAPPER)/include/
#LIBS += -L$$(MACE_DIGIMESH_WRAPPER)/lib -lMACEDigiMeshWrapper

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/release/ -lMACEDigiMeshWrapper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/debug/ -lMACEDigiMeshWrapper
else:unix:!macx: LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/ -lMACEDigiMeshWrapper

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/release/ -lcommsMACE
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/debug/ -lcommsMACE
else:unix: LIBS += -L$$OUT_PWD/../commsMACE/ -lcommsMACE

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/release/ -lcommsMACEHelper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/debug/ -lcommsMACEHelper
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMACEHelper/ -lcommsMACEHelper

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/release/ -ldata_generic_state_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item/debug/ -ldata_generic_state_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item/ -ldata_generic_state_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/release/ -ldata_generic_state_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/debug/ -ldata_generic_state_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_state_item_topic/ -ldata_generic_state_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/release/ -ldata_generic_command_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/debug/ -ldata_generic_command_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/ -ldata_generic_command_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MACE/release/ -ldata_interface_MACE
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MACE/debug/ -ldata_interface_MACE
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_interface_MACE/ -ldata_interface_MACE

SUBDIRS += \
    controllers.pro

DISTFILES += \
    actions/readme.md
