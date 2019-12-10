#-------------------------------------------------
#
# Project created by QtCreator 2017-01-31T19:28:14
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = module_resource_task_allocation
TEMPLATE = lib

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


DEFINES += MODULE_RESOURCE_TASK_ALLOCATION_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += module_rta.cpp \
            environment_custom.cpp \
            polySplit/polygon.cpp \
            polySplit/line.cpp \
            polySplit/polysplit.cpp

HEADERS += module_rta.h\
        module_resource_task_allocation_global.h \
        environment_custom.h \
        polySplit/vector.h \
        polySplit/polygon.h \
        polySplit/line.h \
        polySplit/polysplit.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_resource_task_allocation.lib release/module_resource_task_allocation.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_resource_task_allocation.lib debug/module_resource_task_allocation.dll
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


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/release/ -ldata_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/debug/ -ldata_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_sensors/ -ldata_vehicle_sensors

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../voropp/release/ -lvoropp
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../voropp/debug/ -lvoropp
#else:unix:!macx: LIBS += -L$$OUT_PWD/../voropp/ -lvoropp

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

