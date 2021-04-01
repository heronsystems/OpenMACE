#-------------------------------------------------
#
# Project created by QtCreator 2018-09-06T15:57:43
#
#-------------------------------------------------

QT       -= gui
QT       += core # needed by MaceCore

TARGET = data_auctioneer
TEMPLATE = lib

DEFINES += DATA_AUCTIONEER_LIBRARY

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

SOURCES += \
    assignment_task_queue.cpp \
    auction_logger.cpp \
    auction_statistics_agent.cpp \
    available_task_queue.cpp \
    awarded_task_queue.cpp \
    master_task_queue.cpp \
    scrub_message_topic.cpp \
    task_container.cpp \
    bids/bid_bundle.cpp \
    bids/bid_bundle_topic.cpp \
    bids/bid_container.cpp \
    bids/bid_descriptor.cpp \
    bids/bid_descriptor_topic.cpp \
    state/vehicle_state.cpp \
    auction_statistics_controller.cpp \
    task_status_topic.cpp

HEADERS += \
    assignment_task_queue.h \
    auction_logger.h \
    auction_statistics_agent.h \
    auction_statistics_controller.h \
    auctioneer_components.h \
    available_task_queue.h \
    awarded_task_queue.h \
    data_auctioneer_global.h \
    master_task_queue.h \
    scrub_message.h \
    scrub_message_topic.h \
    task_container.h \
    bids/bid_bundle.h \
    bids/bid_bundle_topic.h \
    bids/bid_container.h \
    bids/bid_descriptor.h \
    bids/bid_descriptor_topic.h \
    state/vehicle_state.h \
    task_status_topic.h



# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/data_auctioneer.lib release/data_auctioneer.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/data_auctioneer.lib debug/data_auctioneer.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


#Necessary header includes
INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/V2/common
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../speedLog/

#Library links
win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_tasks/release/ -ldata_tasks
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_tasks/debug/ -ldata_tasks
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_tasks/ -ldata_tasks


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

SUBDIRS += \
    data_auctioneer.pro

