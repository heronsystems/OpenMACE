#include <stdlib.h>
#include <ros/ros.h>
#include "transform_sensors.hpp"

/**
 * @brief TransfromSensors constructor
 */
TransformSensors::TransformSensors(std::string modelName) : modelName(modelName)
{
    setupPublishers();
    
    m_tfBuffer = std::make_shared<tf2_ros::Buffer>();
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
    m_projector = std::make_shared<laser_geometry::LaserProjection>();
}

/**
 * @brief Setup publishers for when data changes in each callback
 */
void TransformSensors::setupPublishers()
{
    pointCloudPub_laserScan = nh.advertise<sensor_msgs::PointCloud2>("/MACE/" + modelName + "/scan/cloud_global", 1000);
    pointCloudPub_kinect = nh.advertise<sensor_msgs::PointCloud2>("/MACE/" + modelName + "/kinect/depth/points", 1000);
    // laserScanPub = nh.advertise<sensor_msgs::LaserScan>("/MACE/" + modelName + "/scan", 1000);
}

/**
 * @brief Callback function for the laser scans reported from turtlebot
 */
void TransformSensors::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 cloud;
    //std::string modelName = msg->header.frame_id;
    //modelName = modelName.substr(0, modelName.find("/"));
    //std::string transformFrame = modelName + "/base_link";
    std::string transformFrame = "world";
    m_projector->transformLaserScanToPointCloud(transformFrame, *msg, cloud, *m_tfBuffer);

    std::cout << "__Frames__" << std::endl;
    ROS_WARN("Frame for cloud in: %s", msg->header.frame_id.c_str());
    ROS_WARN("Frame for cloud out: %s", cloud.header.frame_id.c_str());
    std::cout << std::endl;

    pointCloudPub_laserScan.publish(cloud);
}

/**
 * @brief Callback function for the kinect/depth/points from simulated UxV
 */
void TransformSensors::kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 cloud_in, cloud_out;
    cloud_in = *msg;
    geometry_msgs::TransformStamped transformStamped;

    /* 
    ****************************************************************
    // TODO-PAT: There seems to be a memory leak in lookupTransform. 
    //           Can't really spawn two vehicles or run one with a
    //           kinect and run this node for any amount of time 
    //           before losing memory and going to SWP
    **************************************************************** 
    */

    try
    {
        transformStamped = m_tfBuffer->lookupTransform("world", msg->header.frame_id, ros::Time(0), ros::Duration(1));
        // std::cout << transformStamped << std::endl;
        tf2::doTransform(cloud_in, cloud_out, transformStamped);

        std::cout << "__Frames__" << std::endl;
        ROS_WARN("Frame for cloud in: %s", cloud_in.header.frame_id.c_str());
        ROS_WARN("Frame for cloud out: %s", cloud_out.header.frame_id.c_str());
        std::cout << std::endl;
    
        // Publish transformed point cloud:
        pointCloudPub_kinect.publish(cloud_out);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}
