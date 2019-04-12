#ifndef MODULE_ROS_H
#define MODULE_ROS_H

#include <iostream>
#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include "common/common.h"
#include "common/background_tasks.h"
#include "module_ROS_global.h"

#include "mace_core/i_module_command_ROS.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "maps/map_topic_components.h"

#include "base/pose/orientation_3D.h"

#include <memory>

#ifdef ROS_EXISTS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/SetModelState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <octomap_ros/conversions.h>

#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
#endif

#include "rosTimer.h"

#include <chrono>
#include <ctime>



class MODULE_ROSSHARED_EXPORT ModuleROS : public MaceCore::IModuleCommandROS
{

public:
    //!
    //! \brief ModuleROS Default constructor
    //!
    ModuleROS();

    ~ModuleROS();

    //!
    //! \brief start Start ROS loop
    //!
    void start();

    // ============================================================================= //
    // ================= Default methods for module configuration ================== //
    // ============================================================================= //
public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
    {
        ptr->Subscribe(this, m_PlanningStateTopic.Name());
        ptr->Subscribe(this, m_VehicleDataTopic.Name());
        ptr->Subscribe(this, m_MapTopic.Name());
    }

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());


    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGenericBoundaries ==== //
    // ============================================================================= //

public:

    //!
    //! \brief NewlyAvailableBoundary Subscriber to a new boundary
    //! \param key Key corresponding to the updated boundary in the core
    //!
    void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;



    // ============================================================================= //
    // ======== Virtual functions as defined by IModuleCommandGroundStation ======== //
    // ============================================================================= //
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    virtual void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief NewlyUpdated3DOccupancyMap Subscriber to a newly available 3D occupancy map
    //!
    void NewlyUpdated3DOccupancyMap() override;

    //!
    //! \brief NewlyCompressedOccupancyMap Subscriber to a newly available compressed occupancy map
    //! \param map Compressed occupancy map
    //!
    void NewlyCompressedOccupancyMap(const mace::maps::Data2DGrid<OccupiedResult> &map) override;


    //!
    //! \brief NewlyFoundPath Subscriber to a new path for a vehicle
    //! \param path Path object
    //!
    void NewlyFoundPath(const std::vector<mace::state_space::StatePtr> &path) override;


    // ============================================================================= //
    // ======================== Module specific functions: ========================= //
    // ============================================================================= //
public:

    void setPublishers(std::string unparsedPublishers);

    //!
    //! \brief insertVehicleIfNotExist Insert a new vehicle into the map if it does not exist
    //! \param vehicleID ID of vehicle to check against current vehicle map
    //!
    void insertVehicleIfNotExist(const int &vehicleID);

    //!
    //! \brief updatePositionData Update the position of the corresponding Gazebo model based on position of MACE vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \param component Position (in the local frame)
    //!
    void updatePositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateLocalPositionTopic> &component);

    //!
    //! \brief updateAttitudeData Update the attitude of the corresponding Gazebo model based on attitude of MACE vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \param component Attitude
    //!
    void updateAttitudeData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateAttitudeTopic> &component);

    // ============================================================================= //
    // ========================  ROS Specific functions:  ========================== //
    // ============================================================================= //
public:

#ifdef ROS_EXISTS
    //!
    //! \brief setupROS Setup ROS subscribers, publishers, and node handler
    //!
    void setupROS();

    //!
    //! \brief newLaserScan Laser scan callback for ROS LaserScan message
    //! \param event LaserScan message
    //!
    void newLaserScan(const ros::MessageEvent<sensor_msgs::LaserScan const>& event);

    //!
    //! \brief newPointCloud Point cloud callback for ROS PointCloud2 message
    //! \param msg PointCloud2 message
    //!
    void newPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    //!
    //! \brief newGlobalPointCloud Point cloud callback for ROS PointCloud2 message (converted to global frame)
    //! \param msg PointCloud2 message
    //!
    void newGlobalPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    //!
    //! \brief convertToGazeboCartesian Convert position in local frame to Gazebo's world frame
    //! \param localPos MACE local position
    //!
    void convertToGazeboCartesian(DataState::StateLocalPosition& localPos);

    //!
    //! \brief sendGazeboModelState Send the current position and attitude of the corresponding vehicle model to Gazebo
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool sendGazeboModelState(const int &vehicleID);

    //!
    //! \brief publishVehiclePose Publish the current position and attitude of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehiclePose(const int &vehicleID);


#endif

    // ============================================================================= //
    // =====================  ROS Specific private functions:  ===================== //
    // ============================================================================= //
#ifdef ROS_EXISTS

    //!
    //! \brief generateColorHeight Assign a RGBA color based on data height for rendering
    //! \param height Height of data point
    //! \return ROS ColorRGBA value
    //!
    std_msgs::ColorRGBA generateColorHeight(double height);


    //!
    //! \brief renderOccupancyMap Render occupancy map in RViz
    //! \param tree OcTree to render
    //!
    void renderOccupancyMap(const std::shared_ptr<octomap::OcTree> &tree);


    //!
    //! \brief renderState Publish the 2D Cartesian Position to ROS for rendering in RViz
    //! \param state 2D Cartesian Position to render
    //!
    void renderState(const mace::pose::CartesianPosition_2D &state);

    //!
    //! \brief renderEdge Publish the 2D line to ROS for rendering in RViz
    //! \param edge Edge/line to render
    //!
    void renderEdge(const mace::geometry::Line_2DC &edge);

#endif

    // ============================================================================= //
    // ========================  Module private members:  ========================== //
    // ============================================================================= //
private:
    //!
    //! \brief m_vehicleMap Container for map of vehicle IDs and corresponding most recent Position and Attitude data
    //!
    std::map<int, std::tuple<DataState::StateLocalPosition, DataState::StateAttitude> > m_vehicleMap;

    //!
    //! \brief m_timer Timer that triggers a ROS spin event to cycle through any queued ROS messages
    //!
    std::shared_ptr<ROSTimer> m_timer;

    //!
    //! \brief m_vehicleID Vehicle ID attached to this ROS module instance
    //!
    int m_vehicleID;

    //!
    //! \brief airborneInstance Flag denoting if this ROS module is attached to an airborne instance
    //!
    bool airborneInstance;

    //!
    //! \brief m_sensors List of sensors that will be spawned into the ROS environment
    //!
    std::vector<std::tuple<std::string, std::string> > m_sensors;

    //!
    //! \brief m_publishers List of publishers that will be published per vehicle
    //!
//    std::vector<std::tuple<std::string, std::string> > m_publishers;

    // ============================================================================= //
    // =====================  ROS Specific private members:  ======================= //
    // ============================================================================= //
#ifdef ROS_EXISTS
    //!
    //! \brief nh ROS node handler
    //!
    ros::NodeHandle nh;

    //!
    //! \brief laserSub Subscriber for ROS laser scan messages
    //!
    ros::Subscriber laserSub;

    //!
    //! \brief pointCloudSub Subscriber for ROS point cloud messages
    //!
    ros::Subscriber pointCloudSub;

    //!
    //! \brief m_sensorVehicleMap Map of the sensors applicable to each vehicle ID
    //!
    std::map<int, std::vector<ros::Subscriber> > m_sensorVehicleMap;

    //!
    //! \brief m_vehiclePosePubMap Map of the pose publisher applicable to each vehicle ID
    //!
    std::map<int, ros::Publisher> m_vehiclePosePubMap;

    //!
    //! \brief markerPub Publisher for markers to be rendered in RViz
    //!
    ros::Publisher markerPub;

    //!
    //! \brief operationalBoundaryPub Publisher for operational boundary to be rendered in RViz
    //!
    ros::Publisher operationalBoundaryPub;

    //!
    //! \brief compressedMapPub Publisher for the compressed map to be rendered in RViz
    //!
    ros::Publisher compressedMapPub;

    //!
    //! \brief octomapPub Publisher handling the occupied voxels of the octomap
    //!
    ros::Publisher occupancyMapPub;

    //!
    //! \brief points Marker containers
    //!
    visualization_msgs::Marker points, line_strip, line_list, path_list, boundary_list;

    //!
    //! \brief m_client Service client for publishing update model state service to Gazebo
    //!
    ros::ServiceClient m_client;

    //!
    //! \brief m_broadcaster ROS TF broadcaster for coordinate transformations
    //!
    tf::TransformBroadcaster m_broadcaster;

    // TODO: Do I need this? Or can I just create a transform before sending the model state to Gazebo?
    //!
    //! \brief m_transform Container for transform between vehicles and the world frame
    //!
    tf::Transform m_transform;

    tf::Transform m_WorldToMap;
    //!
    //! \brief m_srv Container for the Gazebo send model state message
    //!
    gazebo_msgs::SetModelState m_srv;

    //!
    //! \brief m_tfBuffer Container for a tf2 buffer
    //!
    tf2_ros::Buffer m_tfBuffer;

    //!
    //! \brief m_tfListener Container for a tf transform listener for coordinate frame transformations
    //!
    tf::TransformListener m_tfListener;

    // TESTING:
    ros::Publisher cloudInPub;
    // END TESTING
#endif

private:
    Data::TopicDataObjectCollection<BASE_GEOMETRY_TOPICS, BASE_POSE_TOPICS> m_PlanningStateTopic;
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<MAP_DATA_TOPICS> m_MapTopic;

    BackgroundTasks<std::shared_ptr<mace::maps::Data2DGrid<OccupiedResult> >> m_CompressedMapCalculation;
    BackgroundTasks<std::shared_ptr<octomap::OcTree>> m_OccupancyMapCalculation;
};

#endif // MODULE_ROS_H
