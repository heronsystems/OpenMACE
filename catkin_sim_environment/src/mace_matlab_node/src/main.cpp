#include <stdlib.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <thread>
#include <geometry_msgs/Pose.h>

#include <mace_matlab/UPDATE_POSITION.h>
#include <mace_matlab/UPDATE_ATTITUDE.h>
#include <mace_matlab/UPDATE_HEARTBEAT.h>
#include <mace_matlab/UPDATE_CMD_STATUS.h>
#include <mace_matlab/UPDATE_BATTERY.h>
#include <mace_matlab/UPDATE_GPS.h>
#include <mace_matlab/UPDATE_VEHICLE_TARGET.h>

#include <matlab_listener.h>
#include <mace_listener.h>

int main(int argc, char **argv)
{
    // Initialize ROS and name our node "mace_matlab"
    ros::init(argc, argv, "mace_matlab_node");

    // Handle for the process node. Will handle initialization and
    //   cleanup of the node
    ros::NodeHandle nh;
    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);


    // *************************** //
    // ***** Setup services: ***** //
    // *************************** //
    // Instantiate MATLAB listener:
    MATLABListener matlabListener;

    ros::ServiceServer takeoffService = nh.advertiseService("command_takeoff", &MATLABListener::commandTakeoff, &matlabListener);

    ros::ServiceServer landService = nh.advertiseService("command_land", &MATLABListener::commandLand, &matlabListener);

    ros::ServiceServer armService = nh.advertiseService("command_arm", &MATLABListener::commandArm, &matlabListener);

    ros::ServiceServer datumService = nh.advertiseService("command_datum", &MATLABListener::commandDatum, &matlabListener);

    ros::ServiceServer wptService = nh.advertiseService("command_waypoint", &MATLABListener::commandWaypoint, &matlabListener);


    // *************************** //
    // **** Setup publishers: **** //
    // *************************** //
    ros::Publisher vehiclePosPub = nh.advertise<mace_matlab::UPDATE_POSITION> ("/ROS/UPDATE_POSITION", 1000);

    ros::Publisher vehicleAttPub = nh.advertise<mace_matlab::UPDATE_ATTITUDE> ("/ROS/UPDATE_ATTITUDE", 1000);

    ros::Publisher gpsPub = nh.advertise<mace_matlab::UPDATE_GPS> ("/ROS/UPDATE_GPS", 1000);

    ros::Publisher heartbeatPub = nh.advertise<mace_matlab::UPDATE_HEARTBEAT> ("/ROS/UPDATE_HEARTBEAT", 1000);

    ros::Publisher batteryPub = nh.advertise<mace_matlab::UPDATE_BATTERY> ("/ROS/UPDATE_BATTERY", 1000);

    ros::Publisher vehicleTargetPub = nh.advertise<mace_matlab::UPDATE_VEHICLE_TARGET> ("/ROS/UPDATE_VEHICLE_TARGET", 1000);

// Don't think this is needed, as services request a response at send time:
    ros::Publisher cmdStatusPub = nh.advertise<mace_matlab::UPDATE_CMD_STATUS> ("/ROS/UPDATE_CMD_STATUS", 1000);     


    // *************************** //
    // **** Setup subcribers: **** //
    // *************************** //      
    // Instantiate MACE listener:
    MACEListener maceListener;

    ros::Subscriber positionPub = nh.subscribe<mace_matlab::UPDATE_POSITION>("MACE/UPDATE_POSITION", 500, &MACEListener::positionCallback, &maceListener);

    ros::Subscriber attitudeSub = nh.subscribe<mace_matlab::UPDATE_ATTITUDE>("MACE/UPDATE_ATTITUDE", 500, &MACEListener::attitudeCallback, &maceListener);

    ros::Subscriber heartbeatSub = nh.subscribe<mace_matlab::UPDATE_HEARTBEAT>("MACE/UPDATE_HEARTBEAT", 500, &MACEListener::heartbeatCallback, &maceListener);

    ros::Subscriber gpsSub = nh.subscribe<mace_matlab::UPDATE_GPS>("MACE/UPDATE_GPS", 500, &MACEListener::gpsCallback, &maceListener);

    ros::Subscriber batterySub = nh.subscribe<mace_matlab::UPDATE_BATTERY>("MACE/UPDATE_BATTERY", 500, &MACEListener::batteryCallback, &maceListener);

    ros::Subscriber cmdStatusSub = nh.subscribe<mace_matlab::UPDATE_CMD_STATUS>("MACE/UPDATE_CMD_STATUS", 500, &MACEListener::cmdStatusCallback, &maceListener);

    ros::Subscriber vehicleTargetSub = nh.subscribe<mace_matlab::UPDATE_VEHICLE_TARGET>("/MACE/TARGET_STATUS", 500, &MACEListener::vehicleTargetCallback, &maceListener);


    // TEST MESSAGE LOOP:
    // mace_matlab::UPDATE_POSE pose;
    // pose.vehicleID = 1;
    // while(ros::ok()) {
    //     ROS_WARN("Publish message for vehicle: %d", pose.vehicleID);
    //     vehiclePosePub.publish(pose);

    //     ros::spinOnce();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }

    ros::spin();

    return 0;
}
