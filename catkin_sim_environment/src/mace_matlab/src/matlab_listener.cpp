#include <stdlib.h>
#include <ros/ros.h>
#include "matlab_listener.h"

/**
 * @brief MATLABListener constructor
 */
MATLABListener::MATLABListener()
{
}


bool MATLABListener::commandTakeoff(mace_matlab::CMD_TAKEOFF::Request  &req, 
                                    mace_matlab::CMD_TAKEOFF::Response &res)
{
    res.success = true;
 
    ROS_WARN("Command TAKEOFF vehicle ID: %d", req.vehicleID);
    ROS_WARN("sending back response: [%d]", res.success);
    return true;
}

bool MATLABListener::commandArm(mace_matlab::CMD_ARM::Request  &req, 
                                mace_matlab::CMD_ARM::Response &res)
{
    res.success = true;
 
    ROS_WARN("Command ARM vehicle ID: %d", req.vehicleID);
    ROS_WARN("sending back response: [%d]", res.success);
    return true;
}

bool MATLABListener::commandLand(mace_matlab::CMD_LAND::Request  &req, 
                                 mace_matlab::CMD_LAND::Response &res)
{
    res.success = true;
 
    ROS_WARN("Command LAND vehicle ID: %d", req.vehicleID);
    ROS_WARN("sending back response: [%d]", res.success);
    return true;
}

bool MATLABListener::commandWaypoint(mace_matlab::CMD_WPT::Request  &req, 
                                     mace_matlab::CMD_WPT::Response &res)
{
    res.success = true;
 
    ROS_WARN("Command WPT vehicle ID: %d", req.vehicleID);
    ROS_WARN("sending back response: [%d]", res.success);
    return true;
}

bool MATLABListener::commandDatum(mace_matlab::CMD_DATUM::Request  &req, 
                                  mace_matlab::CMD_DATUM::Response &res)
{
    res.success = true;
 
    ROS_WARN("Request Arm vehicle ID: %d", req.vehicleID);
    ROS_WARN("sending back response: [%d]", res.success);
    return true;
}