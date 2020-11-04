#ifndef MODULE_ROS_UMD_H
#define MODULE_ROS_UMD_H

#include <iostream>
#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include "common/common.h"
#include "common/background_tasks.h"
#include "module_ROS_UMD_global.h"

#include "mace_core/i_module_command_ROS.h"

#include "data/topic_data_object_collection.h"
#include "base_topic/base_topic_components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "maps/map_topic_components.h"

#include "base/pose/rotation_3D.h"

#include <memory>

#ifdef ROS_EXISTS
#include <ros/ros.h>
#include <module_ROS_UMD/matlab_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#endif

#include "rosTimer.h"

#include <chrono>
#include <ctime>



class MODULE_ROS_UMDSHARED_EXPORT ModuleROSUMD : public MaceCore::IModuleCommandROS
{

public:
    //!
    //! \brief ModuleROS Default constructor
    //!
    ModuleROSUMD();

    ~ModuleROSUMD() override;

    //!
    //! \brief start Start ROS loop
    //!
    void start() override;

    // ============================================================================= //
    // ================= Default methods for module configuration ================== //
    // ============================================================================= //
public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) override
    {
        ptr->Subscribe(this, m_VehicleDataTopic.Name());
        ptr->Subscribe(this, m_VehicleMissionTopic.Name());
    }

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const override;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params) override;

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
    // ======== Virtual functions as defined by IModuleCommandROS ======== //
    // ============================================================================= //
public:

    //!
    //! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
    //! \param vehicleID Vehilce ID of the newly available vehicle
    //!
    void NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

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

    //!
    //! \brief updatePositionData Update the position of the corresponding Gazebo model based on position of MACE vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \param component Position (in the local frame)
    //!
    void updatePositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> &component);

    //!
    //! \brief updateGlobalPositionData Update the position of the corresponding vehicle and convert to a local position (from Geodetic 3D)
    //! \param vehicleID ID of the vehicle to update
    //! \param component Position (in a global, Geodetic frame)
    //!
    void updateGlobalPositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> &component);

    //!
    //! \brief updateTranslationalVelocity
    //! \param vehicleID
    //! \param component
    //!
    void updateTranslationalVelocity(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_CartesianVelocity> &component);

    //!
    //! \brief updateAttitudeData Update the attitude of the corresponding Gazebo model based on attitude of MACE vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \param component Attitude
    //!
    void updateAttitudeData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> &component);


    //!
    //! \brief updateRotationalVelocity
    //! \param vehicleID
    //! \param component
    //!
    void updateRotationalVelocity(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_RotationalVelocity> &component);

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
    //! \brief publishVehiclePosition Publish the current position of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehiclePosition(const int &vehicleID);

    //!
    //! \brief publishVehicleAttitude Publish the current attitude of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehicleAttitude(const int &vehicleID);

    //!
    //! \brief publishVehicleGPS Publish the current GPS status of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component);

    //!
    //! \brief publishVehicleBattery Publish the current battery status of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehicleBattery(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component);

    //!
    //! \brief publishVehicleHeartbeat Publish the current heartbeat status of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component);

    //!
    //! \brief publishVehicleTargetInfo Publish current info on how the vehicle is progressing to a current target
    //! \param vehicleID ID of the vehicle generating the update
    //! \param component topic data to be published to the MATLAB instace
    //! \return True for success, False for failure
    //!
    bool publishVehicleTargetInfo(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component);

    // Don't think this is needed, as ROS services require a response
    //!
    //! \brief publishCmdStatus Publish the current command status of the corresponding vehicle
    //! \param vehicleID ID of the vehicle to update
    //! \return True for success, False for failure
    //!
    bool publishCmdStatus(const int &vehicleID);

    void publicVehicleOdometry(const int &vehicleID);

public:
    void ROSCallback_VisionPoseEstimate(const geometry_msgs::PoseStamped::ConstPtr &msg);


#endif

    // ============================================================================= //
    // =====================  ROS Specific private functions:  ===================== //
    // ============================================================================= //
#ifdef ROS_EXISTS

#endif

    // ============================================================================= //
    // ========================  Module private members:  ========================== //
    // ============================================================================= //
private:
    //!
    //! \brief m_vehicleMap Container for map of vehicle IDs and corresponding most recent Position and Attitude data
    //!
    std::map<int, std::tuple<mace::pose::CartesianPosition_3D, mace::pose::Rotation_3D>> m_vehicleMap;

    //!
    //! \brief m_timer Timer that triggers a ROS spin event to cycle through any queued ROS messages
    //!
    std::shared_ptr<ROSTimer> m_timer;


    //!
    //! \brief airborneInstance Flag denoting if this ROS module is attached to an airborne instance
    //!
    bool airborneInstance;

    // ============================================================================= //
    // =====================  ROS Specific private members:  ======================= //
    // ============================================================================= //
#ifdef ROS_EXISTS

    //!
    //! \brief m_vehicleMap Container for map of vehicle IDs and corresponding most recent Position and Attitude data
    //!
    std::map<int, nav_msgs::Odometry*> m_vehiclePoseMap;

    //!
    //! \brief nh ROS node handler
    //!
    ros::NodeHandle nh;

    //!
    //! \brief m_matlabListener Container for MATLAB listener and accompanying methods for services
    //!
    std::shared_ptr<MATLABListener> m_matlabListener;

    //!
    //! \brief m_client Service client for arm commands issued from MATLAB
    //!
    ros::ServiceServer m_armService;

    //!
    //! \brief m_client Service client for datum commands issued from MATLAB
    //!
    ros::ServiceServer m_datumService;

    //!
    //! \brief m_client Service client for home commands issued from MATLAB
    //!
    ros::ServiceServer m_homeService;

    //!
    //! \brief m_client Service client for dynamic kinematic commands issued from MATLAB
    //!
    ros::ServiceServer m_dynamicTargetService_Kinematic;

    //!
    //! \brief m_client Service client for dynamic orientation based on Euler commands issued from MATLAB
    //!
    ros::ServiceServer m_dynamicTargetService_OrientationEuler;

    //!
    //! \brief m_client Service client for dynamic orientation based on Euler commands issued from MATLAB
    //!
    ros::ServiceServer m_dynamicTargetService_OrientationQuat;

    //!
    //! \brief m_client Service client for land commands issued from MATLAB
    //!
    ros::ServiceServer m_landService;

    //!
    //! \brief m_client Service client for takeoff commands issued from MATLAB
    //!
    ros::ServiceServer m_takeoffService;

    //!
    //! \brief m_client Service client for waypoint commands issued from MATLAB
    //!
    ros::ServiceServer m_wptService;

    //!
    //! \brief m_vehiclePosPub Publisher for vehicle position
    //!
    ros::Publisher m_vehicleLocalPosPub;
    ros::Publisher m_vehicleGeodeticPosPub;

    //!
    //! \brief m_vehicleAttPub Publisher for vehicle attitude
    //!
    ros::Publisher m_vehicleAttPub;

    //!
    //! \brief m_gpsPub Publisher for gps status
    //!
    ros::Publisher m_gpsPub;

    //!
    //! \brief m_heartbeatPub Publisher for vehicle heartbeat
    //!
    ros::Publisher m_heartbeatPub;

    //!
    //! \brief m_vehicleTargetPub Publisher for status of the vehicle target position
    //!
    ros::Publisher m_vehicleTargetPub;

    //!
    //! \brief m_batteryPub Publisher for vehicle battery status
    //!
    ros::Publisher m_batteryPub;

// Don't think this is needed, as services request a response at send time:
    //!
    //! \brief m_cmdStatusPub Publisher for command status
    //!
    ros::Publisher m_cmdStatusPub;

    ros::Publisher m_posePub;


    ros::Subscriber m_subscriber_VisionPoseEstimate;

    double count = 0.0;

#endif

private:

    //Subscribing to generic vehicle state data that is published throughout the MACE network
    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, BASE_POSE_TOPICS> m_VehicleDataTopic;

    //Subscribing to generic mission topics published throughout the MACE network
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMissionTopic;
};

#endif // MODULE_ROS_UMD_H
