#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>

#include <memory>

/**
 * @brief TransformSensors class handles transforming sensor data into the appropriate frame
 */
class TransformSensors
{
  public:
    /**
     * @brief TransfromSensors constructor
     */
    TransformSensors(std::string modelName);

    /**
     * @brief Callback function for the laser scans reported from simulated UxV
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    /**
     * @brief Callback function for the kinect/depth/points from simulated UxV
     */
    void kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  private:
    /**
     * @brief Setup publishers for when data changes in each callback
     */
    void setupPublishers();

  private:
    /**
     * @brief Name of the model to publish/subscribe to
     */
    std::string modelName;

    /**
     * @brief container for a ROS node handler
     */
    ros::NodeHandle nh;

    /**
     * @brief container for a ROS publisher to publish point cloud data after transform (from Kinect)
     */
    ros::Publisher pointCloudPub_kinect;

    /**
     * @brief container for a ROS publisher to publish point cloud data after transform (from laser scan)
     */
    ros::Publisher pointCloudPub_laserScan;

    /**
     * @brief container for a ROS publisher to publish laser scan after transform
     */
    ros::Publisher laserScanPub;

    /**
     * @brief pointer to a ROS tf2 buffer
     */
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;

    /**
     * @brief pointer to a ROS tf2 transform listener
     */
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    /**
     * @brief pointer to a ROS laser scan helper object
     */
    std::shared_ptr<laser_geometry::LaserProjection> m_projector;
};