QT      += core
QT      -= gui

TARGET = trajectory_control
TEMPLATE = lib

DEFINES += TRAJECTORYCONTROL_LIBRARY

QMAKE_CXXFLAGS += -std=c++14
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

QMAKE_CXXFLAGS_WARN_ON += -Wno-unknown-pragmas

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
  abstract_trajectory_management.cpp \
  tracking_simplified.cpp \
  virtual_target_controller.cpp

HEADERS += \
    abstract_trajectory_management.h \
    tracking_simplified.h \
    trajectory_control_global.h \
    virtual_target_controller.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/trajectory_control.lib release/trajectory_control.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/trajectory_control.lib debug/trajectory_control.dll
INSTALLS += lib


#Header file copy
INSTALL_PREFIX = $$(MACE_ROOT)/include/$$TARGET
INSTALL_HEADERS = $$HEADERS
include(../headerinstall.pri)


INCLUDEPATH += $$PWD/../
INCLUDEPATH += $$(MACE_ROOT)/spdlog/

contains(DEFINES, WITH_HERON_MAVLINK_SUPPORT) {
  message("data_generic_command_item: Compiling with Heron support")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/HeronAI/
}else{
  message("data_generic_command_item: Using standard ardupilot libraries")
  INCLUDEPATH += $$(MACE_ROOT)/tools/mavlink/ardupilot/generated_messages/ardupilotmega/
}

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

# Eigen Warning suppression:
INCLUDEPATH += $$(MACE_ROOT)/Eigen/include/eigen3
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../common/release/ -lcommon
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../common/debug/ -lcommon
else:unix:!macx: LIBS += -L$$OUT_PWD/../common/ -lcommon

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../base/release/ -lbase
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../base/debug/ -lbase
else:unix:!macx: LIBS += -L$$OUT_PWD/../base/ -lbase

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../data/release/ -ldata
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../data/debug/ -ldata
else:unix: LIBS += -L$$OUT_PWD/../data/ -ldata
