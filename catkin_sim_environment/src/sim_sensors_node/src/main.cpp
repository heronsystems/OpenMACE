#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include "transform_sensors.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS and name our node "talker"
    ros::init(argc, argv, "transform_sensors");

    // Strip away the ROS added arguments passed to the node, then parse arguments passed to the node
    std::vector<std::string> args;
    ros::removeROSArgs(argc, argv, args);
    int vehicleID = std::stoi(args[1]);
    std::string modelName = "basic_quadrotor_" + std::to_string(vehicleID);

    // Handle for the process node. Will handle initialization and
    //   cleanup of the node
    ros::NodeHandle nh;

    // TransformSensors object
    TransformSensors sensorTF(modelName);

    // Setup subcribers
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>(modelName + "/scan", 500, &TransformSensors::laserCallback, &sensorTF);
    // Lower queue size because of memory leak (see comment in callback). This will still be an issue as we add more vehicles
    ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(modelName + "/kinect/depth/points", 10, &TransformSensors::kinectDepthCallback, &sensorTF);

    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}