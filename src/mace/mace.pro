TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
QT += serialport
QT += network
QT -= gui

TARGET = MACE
QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += main.cpp \
    data_interpolation.cpp \
    configuration_reader_xml.cpp \
    pugixml.cpp


HEADERS += \
    data_interpolation.h \
    configuration_reader_xml.h \
    pugixml.hpp \
    pugiconfig.hpp \
    module_collection.h \
    doxy_mainpage.h


# Copy Files
target.path = $$(MACE_ROOT)/bin
INSTALLS += target

INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/include
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
INCLUDEPATH += $$PWD/../../spdlog/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MACE/mace_common/
INCLUDEPATH += $$PWD/../../mavlink_cpp/MAVLINK_BASE/ardupilotmega/
INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3
# Octomap Warning suppression:
QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../tools/octomap/bin/ -loctomap -loctomath
else:unix:!macx: LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base_topic/release/ -lbase_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base_topic/debug/ -lbase_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../base_topic/ -lbase_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix:!macx: LIBS += -L$$OUT_PWD/../maps/ -lmaps

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../comms/release/ -lcomms
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../comms/debug/ -lcomms
else:unix: LIBS += -L$$OUT_PWD/../comms/ -lcomms

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/release/ -lcommsMACE
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACE/debug/ -lcommsMACE
else:unix: LIBS += -L$$OUT_PWD/../commsMACE/ -lcommsMACE

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/release/ -lcommsMACEHelper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMACEHelper/debug/ -lcommsMACEHelper
else:unix:!macx: LIBS += -L$$OUT_PWD/../commsMACEHelper/ -lcommsMACEHelper

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/release/ -lcommsMAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../commsMAVLINK/debug/ -lcommsMAVLINK
else:unix: LIBS += -L$$OUT_PWD/../commsMAVLINK/ -lcommsMAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../controllers/release/ -lcontrollers
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../controllers/debug/ -lcontrollers
else:unix:!macx: LIBS += -L$$OUT_PWD/../controllers/ -lcontrollers

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/release/ -ldata_generic_command_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item/debug/ -ldata_generic_command_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item/ -ldata_generic_command_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/release/ -ldata_generic_command_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/debug/ -ldata_generic_command_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_command_item_topic/ -ldata_generic_command_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/release/ -ldata_generic_item
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item/debug/ -ldata_generic_item
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item/ -ldata_generic_item

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/release/ -ldata_generic_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_item_topic/debug/ -ldata_generic_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_item_topic/ -ldata_generic_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/release/ -ldata_generic_mission_item_topic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/debug/ -ldata_generic_mission_item_topic
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_generic_mission_item_topic/ -ldata_generic_mission_item_topic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/release/ -ldata_interface_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/debug/ -ldata_interface_MAVLINK
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_interface_MAVLINK/ -ldata_interface_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/release/ -ldata_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data_vehicle_sensors/debug/ -ldata_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../data_vehicle_sensors/ -ldata_vehicle_sensors

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../mace_core/release/ -lmace_core
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../mace_core/debug/ -lmace_core
else:unix: LIBS += -L$$OUT_PWD/../mace_core/ -lmace_core

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_external_link/release/ -lmodule_external_link
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_external_link/debug/ -lmodule_external_link
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_external_link/ -lmodule_external_link

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/release/ -lmodule_ground_station
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ground_station/debug/ -lmodule_ground_station
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_ground_station/ -lmodule_ground_station

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ml_station/release/ -lmodule_ml_station
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ml_station/debug/ -lmodule_ml_station
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_ml_station/ -lmodule_ml_station

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_resource_task_allocation/release/ -lmodule_resource_task_allocation
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_resource_task_allocation/debug/ -lmodule_resource_task_allocation
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_resource_task_allocation/ -lmodule_resource_task_allocation

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/release/ -lmodule_path_planning_NASAPhase2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/debug/ -lmodule_path_planning_NASAPhase2
else:unix: LIBS += -L$$OUT_PWD/../module_path_planning_NASAPhase2/ -lmodule_path_planning_NASAPhase2

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ROS/release/ -lmodule_ROS
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ROS/debug/ -lmodule_ROS
else:unix: LIBS += -L$$OUT_PWD/../module_ROS/ -lmodule_ROS

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_ROS_UMD/release/ -lmodule_ROS_UMD
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_ROS_UMD/debug/ -lmodule_ROS_UMD
else:unix: LIBS += -L$$OUT_PWD/../module_ROS_UMD/ -lmodule_ROS_UMD

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/release/ -lmodule_vehicle_ardupilot
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/debug/ -lmodule_vehicle_ardupilot
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_ardupilot/ -lmodule_vehicle_ardupilot

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_arducopter/release/ -lmodule_vehicle_arducopter
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_arducopter/debug/ -lmodule_vehicle_arducopter
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_arducopter/ -lmodule_vehicle_arducopter

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_arduplane/release/ -lmodule_vehicle_arduplane
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_arduplane/debug/ -lmodule_vehicle_arduplane
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_arduplane/ -lmodule_vehicle_arduplane

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/release/ -lmodule_vehicle_generic
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_generic/debug/ -lmodule_vehicle_generic
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_generic/ -lmodule_vehicle_generic

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/release/ -lmodule_vehicle_MAVLINK
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/debug/ -lmodule_vehicle_MAVLINK
else:unix: LIBS += -L$$OUT_PWD/../module_vehicle_MAVLINK/ -lmodule_vehicle_MAVLINK

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_sensors/release/ -lmodule_vehicle_sensors
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_sensors/debug/ -lmodule_vehicle_sensors
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_vehicle_sensors/ -lmodule_vehicle_sensors

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_adept/release/ -lmodule_vehicle_adept
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../module_vehicle_adept/debug/ -lmodule_vehicle_adept
else:unix:!macx: LIBS += -L$$OUT_PWD/../module_vehicle_adept/ -lmodule_vehicle_adept

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../planners/release/ -lplanners
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../planners/debug/ -lplanners
else:unix:!macx: LIBS += -L$$OUT_PWD/../planners/ -lplanners

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DigiMesh/release/ -lDigiMesh
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DigiMesh/debug/ -lDigiMesh
else:unix:!macx: LIBS += -L$$OUT_PWD/../DigiMesh/ -lDigiMesh

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/release/ -lMACEDigiMeshWrapper
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/debug/ -lMACEDigiMeshWrapper
else:unix:!macx: LIBS += -L$$OUT_PWD/../MACEDigiMeshWrapper/ -lMACEDigiMeshWrapper

win32: LIBS += -limagehlp

unix: LIBS += -lboost_system
unix: LIBS += -llz4


unix {
      exists($$(ROS_ROOT_DIR)/lib/) {


      DEFINES += ROS_EXISTS
      INCLUDEPATH += $$(ROS_ROOT_DIR)/include
      INCLUDEPATH += $$(ROS_ROOT_DIR)/lib

      LIBS += -L$$(ROS_ROOT_DIR)/lib -lroscpp
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lroscpp_serialization
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lrostime
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lxmlrpcpp
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lcpp_common
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lrosconsole_log4cxx
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lrosconsole_backend_interface
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lroslib
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lrospack
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lmessage_filters
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lclass_loader
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lconsole_bridge
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lrosconsole
      LIBS += -L$$(ROS_ROOT_DIR)/lib -limage_transport
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lcv_bridge
      LIBS += -L$$(ROS_ROOT_DIR)/lib -ltf
      LIBS += -L$$(ROS_ROOT_DIR)/lib -ltf2
      LIBS += -L$$(ROS_ROOT_DIR)/lib -ltf2_ros
      LIBS += -L$$(ROS_ROOT_DIR)/lib -lactionlib
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomap_ros
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomap
      LIBS += -L$$(ROS_ROOT_DIR)/lib -loctomath

      #LIBS += -L$$(MACE_ROOT)/catkin_sim_environment/devel/lib/ -lmace_matlab_msgs

      INCLUDEPATH += $$(MACE_ROOT)/catkin_sim_environment/devel/include
      DEPENDPATH += $$(MACE_ROOT)/catkin_sim_environment/devel/include

      # ROS Warning suppression:
      QMAKE_CXXFLAGS += -isystem $$(ROS_ROOT_DIR)/include
      QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/catkin_sim_environment/devel/include
      } else {
      message("ROS root" path has not been detected...)
      INCLUDEPATH += $$OUT_PWD/../../tools/octomap/octomap/include
      LIBS += -L$$OUT_PWD/../../tools/octomap/lib/ -loctomap -loctomath

      # Octomap Warning suppression:
      QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include
      }
}


unix:!macx|win32: LIBS += -L$$PWD/../../tools/flann/build/lib/ -lflann_s

INCLUDEPATH += $$PWD/../../tools/flann/src/cpp
DEPENDPATH += $$PWD/../../tools/flann/src/cpp

# Flann Warning suppression:
QMAKE_CXXFLAGS += -isystem $$PWD/../../tools/flann/src/cpp
