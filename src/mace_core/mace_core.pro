#-------------------------------------------------
#
# Project created by QtCreator 2016-08-22T13:49:38
#
#-------------------------------------------------
QT       += core
QT       -= gui

TARGET = mace_core
TEMPLATE = lib
win32:TARGET_EXT += .dll

DEFINES += MACE_CORE_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

SOURCES += mace_core.cpp \
    mace_data.cpp \
    module_command_initialization.cpp

HEADERS += mace_core.h\
        mace_core_global.h \
    metadata_vehicle.h \
    metadata_ground_station.h \
    metadata_rta.h \
    i_module_command_RTA.h \
    i_module_command_ground_station.h \
    i_module_command_vehicle.h \
    i_module_events_rta.h \
    i_module_events_ground_station.h \
    i_module_events_vehicle.h \
    vehicle_data.h \
    i_module_command_path_planning.h \
    i_module_command_ROS.h \
    i_module_events_path_planning.h \
    i_module_events_ROS.h \
    metadata_path_planning.h \
    metadata_ROS.h \
    mace_data.h \
    observation_history.h \
    module_parameters.h \
    abstract_module_base.h \
    abstract_module_event_listeners.h \
    module_factory.h \
    abstract_module_base_vehicle_listener.h \
    matrix_operations.h \
    command_marshler.h \
    topic.h \
    i_module_topic_events.h \
    i_module_command_external_link.h \
    i_module_command_sensors.h \
    i_module_events_sensors.h \
    metadata_sensors.h \
    i_module_events_external_link.h \
    i_module_events_general.h \
    i_module_events_general_vehicle.h \
    module_characteristics.h \
    i_module_events_boundary_generator.h \
    i_module_command_generic_boundaries.h \
    i_module_command_generic_vehicle_listener.h


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$PWD/../../speedLog/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/mace_core.lib release/mace_core.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/mace_core.lib debug/mace_core.dll
INSTALLS += lib

#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

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
