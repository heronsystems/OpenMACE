#include <stdlib.h>
#include <ros/ros.h>
#include "mace_listener.h"

/**
 * @brief MACEListener constructor
 */
MACEListener::MACEListener()
{
}


void MACEListener::attitudeCallback(const mace_matlab_msgs::UPDATE_ATTITUDE::ConstPtr &msg) {
    ROS_INFO("ATTITUDE callback for vehicle ID: %d", msg->vehicleID);
}

void MACEListener::batteryCallback(const mace_matlab_msgs::UPDATE_BATTERY::ConstPtr &msg) {
    ROS_INFO("BATTERY callback for vehicle ID: %d", msg->vehicleID);
}

// Not needed: services require a response at time of send
void MACEListener::cmdStatusCallback(const mace_matlab_msgs::UPDATE_CMD_STATUS::ConstPtr &msg) {
    ROS_INFO("CMD_STATUS callback for vehicle ID: %d", msg->vehicleID);
}

void MACEListener::gpsCallback(const mace_matlab_msgs::UPDATE_GPS::ConstPtr &msg) {
    ROS_INFO("GPS callback for vehicle ID: %d", msg->vehicleID);
}

void MACEListener::heartbeatCallback(const mace_matlab_msgs::UPDATE_HEARTBEAT::ConstPtr &msg) {
    ROS_INFO("HEARTBEAT callback for vehicle ID: %d", msg->vehicleID);
}

void MACEListener::positionCallback(const mace_matlab_msgs::UPDATE_POSITION::ConstPtr &msg) {
    ROS_INFO("POSITION callback for vehicle ID: %d", msg->vehicleID);
}

void MACEListener::vehicleTargetCallback(const mace_matlab_msgs::UPDATE_VEHICLE_TARGET::ConstPtr &msg) {
    ROS_INFO("UPDATE VEHICLE TARGET callback for vehicle ID: %d", msg->vehicleID);
}
