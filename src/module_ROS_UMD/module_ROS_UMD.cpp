/**
 *  @file      module_ROS.cpp
 *  @brief     ModuleROSUMD class implementation
 *  @details   Implementation of the ROS module support methods.
 *  @author    Patrick Nolan (patnolan33)
 *  @copyright BSD
 */


#include <module_ROS_UMD.h>

#include "mace_core/module_factory.h"

#include <map>
#include <string>
#include <iostream>

#include <limits>

#include <ctime>

#ifdef ROS_EXISTS
#include <mace_matlab_msgs/UPDATE_ATTITUDE.h>
#include <mace_matlab_msgs/UPDATE_BATTERY.h>
#include <mace_matlab_msgs/UPDATE_CMD_STATUS.h>
#include <mace_matlab_msgs/UPDATE_GPS.h>
#include <mace_matlab_msgs/UPDATE_HEARTBEAT.h>
#include <mace_matlab_msgs/UPDATE_GEODETIC_POSITION.h>
#include <mace_matlab_msgs/UPDATE_LOCAL_POSITION.h>
#include <mace_matlab_msgs/UPDATE_VEHICLE_TARGET.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#endif

//!
//! \brief ModuleROSUMD Default constructor
//!
ModuleROSUMD::ModuleROSUMD() :
    MaceCore::IModuleCommandROS(),
    m_VehicleDataTopic("vehicleData"), m_VehicleMissionTopic("vehicleMission")
{
#ifdef ROS_EXISTS
    m_matlabListener = std::make_shared<MATLABListener>(this);
#endif
}

ModuleROSUMD::~ModuleROSUMD() {
    if(m_timer)
    {
        m_timer->stop();
    }

#ifdef ROS_EXISTS
    ros::shutdown();
#endif
}

//!
//! \brief start Start ROS loop
//!
void ModuleROSUMD::start() {
#ifdef ROS_EXISTS
    this->setupROS();

    // Start timer:
    m_timer = std::make_shared<ROSTimer>([=]()
    {
        // Spin ROS
        ros::spinOnce();
    });

    m_timer->setSingleShot(false);
    m_timer->setInterval(ROSTimer::Interval(33));
    m_timer->start(true);
#endif

    AbstractModule_EventListeners::start();
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleROSUMD::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    structure.AddTerminalParameters("ID", MaceCore::ModuleParameterTerminalTypes::INT, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleROSUMD::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    std::cout << "Configure module UMD ROS" << std::endl;


    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");
    }

    if(params->HasTerminal("ID"))
    {
        this->SetID(params->GetTerminalValue<int>("ID"));
    }
}


//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleROSUMD::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(topicName);
    UNUSED(sender);
    UNUSED(data);
    UNUSED(target);
}


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
void ModuleROSUMD::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);

    uint8_t vehicleID;
    if(this->getDataObject()->getMavlinkIDFromModule(sender, vehicleID)) {
        if(topicName == m_VehicleDataTopic.Name())
        {
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
                if(componentsUpdated.at(i) == mace::pose_topics::Topic_AgentOrientation::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> component = std::make_shared<mace::pose_topics::Topic_AgentOrientation>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    updateAttitudeData(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_RotationalVelocity::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_RotationalVelocity> component = std::make_shared<mace::pose_topics::Topic_RotationalVelocity>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    updateRotationalVelocity(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_CartesianPosition::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> component = std::make_shared<mace::pose_topics::Topic_CartesianPosition>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    updatePositionData(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_CartesianVelocity::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_CartesianVelocity> component = std::make_shared<mace::pose_topics::Topic_CartesianVelocity>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    updateTranslationalVelocity(vehicleID,component);
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_GeodeticPosition::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> component = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    updateGlobalPositionData(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
#ifdef ROS_EXISTS
                    publishVehicleBattery(vehicleID, component);
#endif
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
#ifdef ROS_EXISTS
                    publishVehicleGPS(vehicleID, component);
#endif
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()){
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
#ifdef ROS_EXISTS
                    publishVehicleHeartbeat(vehicleID, component);
#endif
                }
            }
        }
        else if(topicName == m_VehicleMissionTopic.Name())
        {
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                //get latest datagram from mace_data
                MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMissionTopic.Name(), sender);
                if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {

                }
                else if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {

                }
                else if(componentsUpdated.at(i) == MissionTopic::VehicleTargetTopic::Name()) {
                    std::shared_ptr<MissionTopic::VehicleTargetTopic> component = std::make_shared<MissionTopic::VehicleTargetTopic>();
                    m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
#ifdef ROS_EXISTS
                    publishVehicleTargetInfo(vehicleID, component);
#endif
                }
            }
        }
    }
}

//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleROSUMD::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);

#ifdef ROS_EXISTS
    nav_msgs::Odometry* defaultOdom = new nav_msgs::Odometry();
    defaultOdom->child_frame_id = "odom";
    geometry_msgs::Quaternion defaultQuaternion;
    defaultQuaternion.x = 0.0; defaultQuaternion.y = 0.0; defaultQuaternion.z = 0.0; defaultQuaternion.w = 1.0;
    defaultOdom->pose.pose.orientation = defaultQuaternion;

    this->m_vehiclePoseMap.insert(std::pair<int,nav_msgs::Odometry*>(vehicleID,defaultOdom));
#endif
}

//!
//! \brief NewlyUpdated3DOccupancyMap Subscriber to a newly available 3D occupancy map
//!
void ModuleROSUMD::NewlyUpdated3DOccupancyMap()
{
}

//!
//! \brief NewlyCompressedOccupancyMap Subscriber to a newly available compressed occupancy map
//! \param map Compressed occupancy map
//!
void ModuleROSUMD::NewlyCompressedOccupancyMap(const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map)
{
    UNUSED(map);
}

//!
//! \brief NewlyUpdatedOperationalFence Subscriber to a new operational fence (i.e. global boundary)
//! \param boundary Boundary list object in Cartesian space
//!
void ModuleROSUMD::NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key);
    UNUSED(sender);
}

//!
//! \brief NewlyFoundPath Subscriber to a new path for a vehicle
//! \param path Path object
//!
void ModuleROSUMD::NewlyFoundPath(const std::vector<mace::state_space::StatePtr> &path)
{
    UNUSED(path);
}


////! ========================================================================
////! ======================  ROS Specific functions:  =======================
////! ========================================================================

//!
//! \brief updateAttitudeData Update the attitude of the corresponding Gazebo model based on attitude of MACE vehicle
//! \param vehicleID ID of the vehicle to update
//! \param component Attitude
//!
void ModuleROSUMD::updateAttitudeData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_AgentOrientation> &component)
{
    mace::pose::Rotation_3D currentRotation;

    if(component->getRotationObj()->getDOF() == 3)
    {
        mace::pose::Rotation_3D* castRotation = component->getRotationObj()->rotationAs<mace::pose::Rotation_3D>();
        currentRotation = *castRotation;
    }
    else if(component->getRotationObj()->getDOF() == 2)
    {
        mace::pose::Rotation_2D* castRotation = component->getRotationObj()->rotationAs<mace::pose::Rotation_2D>();
        currentRotation.updateYaw(castRotation->getPhi());
    }


    if(m_vehicleMap.find(vehicleID) != m_vehicleMap.end()) {
        std::tuple<mace::pose::CartesianPosition_3D, mace::pose::Rotation_3D> tmpTuple;
        mace::pose::CartesianPosition_3D tmpPos = std::get<0>(m_vehicleMap[vehicleID]);
        tmpTuple = std::make_tuple(tmpPos, currentRotation);
        m_vehicleMap[vehicleID] = tmpTuple;
    }

#ifdef ROS_EXISTS
    // TODO: Publish vehicle Pose to MATLAB
    publishVehicleAttitude(vehicleID);

    std::map<int,nav_msgs::Odometry*>::iterator it;
    nav_msgs::Odometry* currentObject = nullptr;
    it = m_vehiclePoseMap.find(vehicleID);
    if(it != m_vehiclePoseMap.end())
    {
        currentObject = it->second;
        Eigen::Quaterniond currentQuaternion = currentRotation.getQuaternion();
        currentObject->pose.pose.orientation.x = currentQuaternion.x();
        currentObject->pose.pose.orientation.y = currentQuaternion.y();
        currentObject->pose.pose.orientation.z = currentQuaternion.z();
        currentObject->pose.pose.orientation.w = currentQuaternion.w();
        publicVehicleOdometry(vehicleID);
    }

#endif
}


//!
//! \brief updatePositionData Update the position of the corresponding Gazebo model based on position of MACE vehicle
//! \param vehicleID ID of the vehicle to update
//! \param component Position (in the local frame)
//!
void ModuleROSUMD::updatePositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> &component)
{
#ifdef ROS_EXISTS
    mace_matlab_msgs::UPDATE_LOCAL_POSITION position;
    position.vehicleID = vehicleID;

    std::map<int,nav_msgs::Odometry*>::iterator it;
    nav_msgs::Odometry* currentObject = nullptr;
    it = m_vehiclePoseMap.find(vehicleID);
    if(it != m_vehiclePoseMap.end())
        currentObject = it->second;

    if(component->getPositionObj()->is2D())
    {
        mace::pose::CartesianPosition_2D* castPosition = component->getPositionObj()->positionAs<mace::pose::CartesianPosition_2D>();
        position.northing = castPosition->getXPosition();
        position.easting = castPosition->getYPosition();
        if(currentObject != nullptr)
        {
            currentObject->header.stamp = ros::Time::now();
            currentObject->pose.pose.position.x = castPosition->getXPosition();
            currentObject->pose.pose.position.y = castPosition->getXPosition();
        }
    }

    if(component->getPositionObj()->is3D())
    {
        mace::pose::CartesianPosition_3D* castPosition = component->getPositionObj()->positionAs<mace::pose::CartesianPosition_3D>();
        position.northing = castPosition->getXPosition();
        position.easting = castPosition->getYPosition();
        position.altitude = -castPosition->getZPosition();

        if(currentObject != nullptr)
        {
            currentObject->header.stamp = ros::Time::now();
            currentObject->pose.pose.position.x = castPosition->getXPosition();
            currentObject->pose.pose.position.y = castPosition->getYPosition();
            currentObject->pose.pose.position.z = castPosition->getZPosition();
        }

    }
    position.northSpeed = 0.0; // TODO
    position.eastSpeed = 0.0; // TODO
    m_vehicleLocalPosPub.publish(position);

#endif
}

//!
//! \brief updateGlobalPositionData Update the position of the corresponding vehicle and convert to a local position (from Geodetic 3D)
//! \param vehicleID ID of the vehicle to update
//! \param component Position (in a global, Geodetic frame)
//!
void ModuleROSUMD::updateGlobalPositionData(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> &component)
{
#ifdef ROS_EXISTS
    mace_matlab_msgs::UPDATE_GEODETIC_POSITION position;
    position.vehicleID = vehicleID;
    //the command here is based in a global cartesian space, we therefore need to transform it

    if(component->getPositionObj()->is2D())
    {
        mace::pose::GeodeticPosition_2D* castPosition = component->getPositionObj()->positionAs<mace::pose::GeodeticPosition_2D>();
        position.latitude = castPosition->getLatitude();
        position.longitude = castPosition->getLongitude();
    }

    if(component->getPositionObj()->is3D())
    {
        mace::pose::GeodeticPosition_3D* castPosition = component->getPositionObj()->positionAs<mace::pose::GeodeticPosition_3D>();
        position.latitude = castPosition->getLatitude();
        position.longitude = castPosition->getLongitude();
        position.altitude = castPosition->getAltitude();
    }
    // TODO: Publish vehicle Pose to MATLAB
    position.northSpeed = 0.0; // TODO
    position.eastSpeed = 0.0; // TODO
    m_vehicleGeodeticPosPub.publish(position);
#endif
}


//!
//! \brief updateTranslationalVelocity
//! \param vehicleID
//! \param component
//!
void ModuleROSUMD::updateTranslationalVelocity(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_CartesianVelocity> &component)
{
#ifdef ROS_EXISTS
    std::map<int,nav_msgs::Odometry*>::iterator it;
    nav_msgs::Odometry* currentObject = nullptr;
    it = m_vehiclePoseMap.find(vehicleID);
    if(it != m_vehiclePoseMap.end())
        currentObject = it->second;

    if(component->getVelocityObj()->is2D())
    {

    }

    else if(component->getVelocityObj()->is3D())
    {
        mace::pose::Velocity_Cartesian3D* castVelocity = component->getVelocityObj()->velocityAs<mace::pose::Velocity_Cartesian3D>();
        if(currentObject != nullptr)
        {
            currentObject->header.stamp = ros::Time::now();
            currentObject->twist.twist.linear.x = castVelocity->getXVelocity();
            currentObject->twist.twist.linear.y = castVelocity->getYVelocity();
            currentObject->twist.twist.linear.z = castVelocity->getZVelocity();
        }
    }
#endif
}


//!
//! \brief updateRotationalVelocity
//! \param vehicleID
//! \param component
//!
void ModuleROSUMD::updateRotationalVelocity(const int &vehicleID, const std::shared_ptr<mace::pose_topics::Topic_RotationalVelocity> &component)
{
#ifdef ROS_EXISTS
    std::map<int,nav_msgs::Odometry*>::iterator it;
    nav_msgs::Odometry* currentObject = nullptr;
    it = m_vehiclePoseMap.find(vehicleID);
    if(it != m_vehiclePoseMap.end())
        currentObject = it->second;

    mace::pose::Velocity_Rotation3D castVelocity = component->getVelocityObj();
    if(currentObject != nullptr)
    {
        currentObject->header.stamp = ros::Time::now();
        currentObject->twist.twist.angular.x = castVelocity.data.x();
        currentObject->twist.twist.angular.y = castVelocity.data.y();
        currentObject->twist.twist.angular.z = castVelocity.data.z();
    }
#endif
}

#ifdef ROS_EXISTS
//!
//! \brief setupROS Setup ROS subscribers, publishers, and node handler
//!
void ModuleROSUMD::setupROS() {
    // *************************** //
    // ***** Setup services: ***** //
    // *************************** //
    m_armService = nh.advertiseService("command_arm", &MATLABListener::commandArm, m_matlabListener.get());
    m_datumService = nh.advertiseService("command_datum", &MATLABListener::commandDatum, m_matlabListener.get());
    m_homeService = nh.advertiseService("command_home", &MATLABListener::commandHome, m_matlabListener.get());
    m_dynamicTargetService_Kinematic = nh.advertiseService("command_dynamic_target_kinematic", &MATLABListener::commandDynamicTarget, m_matlabListener.get());
    m_dynamicTargetService_OrientationEuler = nh.advertiseService("command_dynamic_target_euler", &MATLABListener::commandDynamicTarget_OrientationEuler, m_matlabListener.get());
    m_dynamicTargetService_OrientationQuat = nh.advertiseService("command_dynamic_target_quat", &MATLABListener::commandDynamicTarget_OrientationQuat, m_matlabListener.get());

    m_landService = nh.advertiseService("command_land", &MATLABListener::commandLand, m_matlabListener.get());
    m_takeoffService = nh.advertiseService("command_takeoff", &MATLABListener::commandTakeoff, m_matlabListener.get());
    m_wptService = nh.advertiseService("command_waypoint", &MATLABListener::commandWaypoint, m_matlabListener.get());

    // *************************** //
    // **** Setup publishers: **** //
    // *************************** //
    m_vehicleLocalPosPub = nh.advertise<mace_matlab_msgs::UPDATE_LOCAL_POSITION> ("/MACE/UPDATE_LOCAL_POSITION", 1);
    m_vehicleGeodeticPosPub = nh.advertise<mace_matlab_msgs::UPDATE_GEODETIC_POSITION> ("/MACE/UPDATE_GEODETIC_POSITION", 1);

    m_vehicleAttPub = nh.advertise<mace_matlab_msgs::UPDATE_ATTITUDE> ("/MACE/UPDATE_ATTITUDE", 1);
    m_gpsPub = nh.advertise<mace_matlab_msgs::UPDATE_GPS> ("/MACE/UPDATE_GPS", 1);
    m_heartbeatPub = nh.advertise<mace_matlab_msgs::UPDATE_HEARTBEAT> ("/MACE/UPDATE_HEARTBEAT", 1);
    m_batteryPub = nh.advertise<mace_matlab_msgs::UPDATE_BATTERY> ("/MACE/UPDATE_BATTERY", 1);
    m_vehicleTargetPub = nh.advertise<mace_matlab_msgs::UPDATE_VEHICLE_TARGET>("/MACE/TARGET_STATUS",1);
    // Don't think this is needed, as services request a response at send time:
    m_cmdStatusPub = nh.advertise<mace_matlab_msgs::UPDATE_CMD_STATUS> ("/MACE/UPDATE_CMD_STATUS", 1);
    m_posePub = nh.advertise<nav_msgs::Odometry> ("/MACE/UPDATE_ODOMETRY", 1);

    // *************************** //
    // **** Setup subscribers: **** //
    // *************************** //
    m_subscriber_VisionPoseEstimate = nh.subscribe("/t265_process/vision_pose", 1, &ModuleROSUMD::ROSCallback_VisionPoseEstimate, this);
    ros::spinOnce();
}

void ModuleROSUMD::ROSCallback_VisionPoseEstimate(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    mace::pose::Pose currentPose;
    currentPose.setTimeNow();
    currentPose.m_Position.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_NED);
    currentPose.m_Position.setXPosition(msg->pose.position.x);
    currentPose.m_Position.setYPosition(msg->pose.position.y);
    currentPose.m_Position.setZPosition(msg->pose.position.z);

    geometry_msgs::Quaternion msgRotation = msg->pose.orientation;
    Eigen::Quaterniond currentRotation;
    currentRotation.x() = msgRotation.x;
    currentRotation.y() = msgRotation.y;
    currentRotation.z() = msgRotation.z;
    currentRotation.w() = msgRotation.w;
    currentPose.m_Rotation.setQuaternion(currentRotation);

    this->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->ROS_NewVisionPoseEstimate(1, currentPose);
    });

}


//!
//! \brief publishVehiclePosition Publish the current position of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishVehiclePosition(const int &vehicleID)
{
    // robot state
    UNUSED(vehicleID);
    return false;
}


//!
//! \brief publishVehicleAttitude Publish the current attitude of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishVehicleAttitude(const int &vehicleID)
{

    // robot state
    mace::pose::Rotation_3D tmpAtt = std::get<1>(m_vehicleMap[vehicleID]);

    mace_matlab_msgs::UPDATE_ATTITUDE attitude;
    attitude.vehicleID = vehicleID;
    attitude.roll = tmpAtt.getRoll();
    attitude.pitch = tmpAtt.getPitch();
    attitude.yaw = tmpAtt.getYaw();
    attitude.rollRate = 0.0;
    attitude.pitchRate = 0.0;
    attitude.yawRate = 0.0;
    // TODO: Timestamp

    m_vehicleAttPub.publish(attitude);

    return true;
}


//!
//! \brief publishVehicleGPS Publish the current GPS status of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishVehicleGPS(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> &component)
{
    mace_matlab_msgs::UPDATE_GPS gps;
    gps.vehicleID = vehicleID;
    gps.gpsFix = DataGenericItem::DataGenericItem_GPS::GPSFixTypeToString(component->getGPSFix());
    gps.numSats = component->getSatVisible();
    gps.hdop = component->getHDOP();
    gps.vdop = component->getVDOP();
    // TODO: Timestamp

    m_gpsPub.publish(gps);

    return true;
}

//!
//! \brief publishVehicleBattery Publish the current battery status of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishVehicleBattery(const int &vehicleID, const std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> &component)
{
    mace_matlab_msgs::UPDATE_BATTERY battery;
    battery.vehicleID = vehicleID;
    battery.voltage = component->getBatteryVoltage();
    battery.current = component->getBatteryCurrent();
    battery.percentRemaining = component->getBatteryRemaining();
    // TODO: Timestamp

    m_batteryPub.publish(battery);

    return true;
}

//!
//! \brief publishVehicleHeartbeat Publish the current heartbeat status of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishVehicleHeartbeat(const int &vehicleID, const std::shared_ptr<DataGenericItem::DataGenericItem_Heartbeat> &component)
{
    mace_matlab_msgs::UPDATE_HEARTBEAT heartbeat;
    heartbeat.vehicleID = vehicleID;
    heartbeat.aircraftType = Data::SystemTypeToString(component->getType());
    heartbeat.autopilot = Data::AutopilotTypeToString(component->getAutopilot());
    heartbeat.isCompanion = component->getCompanion();
    heartbeat.missionState = Data::MissionExecutionStateToString(component->getMissionState());
    // TODO: Timestamp

    m_heartbeatPub.publish(heartbeat);

    return true;
}

bool ModuleROSUMD::publishVehicleTargetInfo(const int &vehicleID, const std::shared_ptr<MissionTopic::VehicleTargetTopic> &component)
{
    UNUSED(vehicleID);
    UNUSED(component);

    return false;

    //    mace_matlab::UPDATE_VEHICLE_TARGET target;
    //    target.vehicleID = vehicleID;
    //    target.state = static_cast<uint8_t>(component->targetState);
    //    target.distance = component->targetDistance;

    //the command here is based in a global cartesian space, we therefore need to transform it
    //    CartesianPosition_3D cartesianPosition;
    //    GeodeticPosition_3D globalOrigin = this->getDataObject()->GetGlobalOrigin();
    //    GeodeticPosition_3D currentPosition;

    //    DynamicsAid::GlobalPositionToLocal(globalOrigin, currentPosition, cartesianPosition);

    //    target.northing = cartesianPosition.getYPosition();
    //    target.easting = cartesianPosition.getXPosition();
    //    target.altitude = cartesianPosition.getZPosition();

    // TODO: Better conversion to ENU:
    //    if(component->targetPosition.getCoordinateFrame() != Data::CoordinateFrameType::CF_LOCAL_ENU) {
    //        target.altitude = -target.altitude;
    //    }

    // TODO: Timestamp

    //    m_vehicleTargetPub.publish(target);
}


// Don't think this is needed, as ROS services require a response
//!
//! \brief publishCmdStatus Publish the current command status of the corresponding vehicle
//! \param vehicleID ID of the vehicle to update
//! \return True for success, False for failure
//!
bool ModuleROSUMD::publishCmdStatus(const int &vehicleID) {
    // Not needed?
    UNUSED(vehicleID);
    return false;
}

void ModuleROSUMD::publicVehicleOdometry(const int &vehicleID)
{
    std::map<int,nav_msgs::Odometry*>::iterator it;
    nav_msgs::Odometry* currentObject = nullptr;
    it = m_vehiclePoseMap.find(vehicleID);
    if(it != m_vehiclePoseMap.end())
        currentObject = it->second;

    m_posePub.publish(*currentObject);
}


#endif
