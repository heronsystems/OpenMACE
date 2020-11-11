#-------------------------------------------------
#
# Project created by QtCreator 2016-08-24T11:42:27
#
#-------------------------------------------------

QT      += core
QT      -= gui

TARGET = module_ROS
TEMPLATE = lib

DEFINES += MODULE_ROS_LIBRARY

QMAKE_CXXFLAGS += -std=c++11
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


SOURCES += module_ROS.cpp \
    rosTimer.cpp

HEADERS += module_ROS.h \
    module_ROS_global.h \
    rosTimer.h

# Unix lib Install
unix:!symbian {
    target.path = $$(MACE_ROOT)/lib
    INSTALLS += target
}

# Windows lib install
lib.path    = $$(MACE_ROOT)/lib
win32:CONFIG(release, debug|release):       lib.files   += release/module_ROS.lib release/module_ROS.dll
else:win32:CONFIG(debug, debug|release):    lib.files   += debug/module_ROS.lib debug/module_ROS.dll
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

# Eigen Warning suppression:
QMAKE_CXXFLAGS += -isystem $$(MACE_ROOT)/Eigen/include/eigen3
# Octomap Warning suppression:
QMAKE_CXXFLAGS += -isystem $$OUT_PWD/../../tools/octomap/octomap/include

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../maps/release/ -lmaps
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../maps/debug/ -lmaps
else:unix: LIBS += -L$$OUT_PWD/../maps/ -lmaps

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

