#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:42:27
#
#-------------------------------------------------

#QT       -= core gui
QT      -= gui
QT      += core

TARGET = module_ROS_UMD
TEMPLATE = lib

DEFINES += MODULE_ROS_UMD_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += module_ROS_UMD.cpp \
    rosTimer.cpp \
    matlab_listener.cpp

HEADERS += module_ROS_UMD.h \
    module_ROS_UMD_global.h \
    rosTimer.h \
    matlab_listener.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_UMD_ROS.lib release/module_ROS_UMD.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_UMD_ROS.lib debug/module_ROS_UMD.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/include
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
DEPENDPATH += $$PWD/../

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix: LIBS += -L$$OUT_PWD/../maps/ -lmaps

unix {
exists(/opt/ros/melodic/lib/) {
    DEFINES += ROS_EXISTS
    INCLUDEPATH += /opt/ros/melodic/include
    INCLUDEPATH += /opt/ros/melodic/lib

        LIBS += -L/opt/ros/melodic/lib -lroscpp
        LIBS += -L/opt/ros/melodic/lib -lroscpp_serialization
        LIBS += -L/opt/ros/melodic/lib -lrostime
        LIBS += -L/opt/ros/melodic/lib -lxmlrpcpp
        LIBS += -L/opt/ros/melodic/lib -lcpp_common
        LIBS += -L/opt/ros/melodic/lib -lrosconsole_log4cxx
        LIBS += -L/opt/ros/melodic/lib -lrosconsole_backend_interface
        LIBS += -L/opt/ros/melodic/lib -lroslib
        LIBS += -L/opt/ros/melodic/lib -lrospack
        LIBS += -L/opt/ros/melodic/lib -lmessage_filters
        LIBS += -L/opt/ros/melodic/lib -lclass_loader
        LIBS += -L/opt/ros/melodic/lib -lconsole_bridge
        LIBS += -L/opt/ros/melodic/lib -lrosconsole
        LIBS += -L/opt/ros/melodic/lib -limage_transport
        LIBS += -L/opt/ros/melodic/lib -lcv_bridge
        LIBS += -L/opt/ros/melodic/lib -ltf
        LIBS += -L/opt/ros/melodic/lib -ltf2
        LIBS += -L/opt/ros/melodic/lib -ltf2_ros
        LIBS += -L/opt/ros/melodic/lib -lactionlib
        LIBS += -L/opt/ros/melodic/lib -loctomap_ros
        LIBS += -L/opt/ros/melodic/lib -loctomap
        LIBS += -L/opt/ros/melodic/lib -loctomath


#        LIBS += -L$$(MACE_ROOT)/catkin_sim_environment/devel/lib/ -lmace_matlab_msgs

        INCLUDEPATH += $$(MACE_ROOT)/catkin_sim_environment/devel/include
        DEPENDPATH += $$(MACE_ROOT)/catkin_sim_environment/devel/include
}
}
