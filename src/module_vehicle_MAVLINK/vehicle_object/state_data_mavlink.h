#ifndef STATE_DATA_MAVLINK_H
#define STATE_DATA_MAVLINK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "data/data_get_set_notifier.h"

#include "base/pose/pose_components.h"
#include "base/measurements/base_speed.h"

#include "base_topic/base_topic_components.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"

class StateData_MAVLINK
{
public:
    StateData_MAVLINK();

    StateData_MAVLINK(const StateData_MAVLINK &copy) = delete;

    ~StateData_MAVLINK() = default;

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetTopicData();

public:
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Heartbeat> vehicleHeartbeat;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_FlightMode> vehicleMode;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_SystemArm> vehicleArm;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Battery> vehicleFuel;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_GPS> vehicleGPSStatus;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Text> vehicleTextAlert;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_SystemTime> vehicleSystemTime;

public:
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> swarmGlobalOrigin;
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalOrigin;
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalHome;

public:
    void set_ShouldTransformLocalAltitude(const bool &transform);
    bool shouldTransformLocalAltitude() const;

private:

    void updatePositionalTransformations_Home();
    void updatePositionalTransformations_EKF();

public:
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalPosition;
    Data::DataGetSetNotifier<mace::pose::CartesianPosition_3D> vehicleLocalPosition;

    Data::DataGetSetNotifier<mace::pose::Velocity_Cartesian3D> vehicleLocalVelocity;

    Data::DataGetSetNotifier<mace::pose::Rotation_3D> vehicleAttitude;
    Data::DataGetSetNotifier<mace::pose::Velocity_Rotation3D> vehicleRotationalVelocity;

    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleAirspeed;
    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleGroundSpeed;

public:
    Eigen::Transform<double,3,Eigen::Affine> getTransform_VehicleHomeTOSwarm() const
    {
        return m_vehicleHomeTOswarm;
    }

    Eigen::Transform<double,3,Eigen::Affine> getTransform_VehicleEKFTOSwarm() const
    {
        return m_vehicleEKFTOswarm;
    }

    Eigen::Transform<double,3,Eigen::Affine> getTransform_SwarmTOVehicleHome() const
    {
        return m_swarmTOvehicleHome;
    }

    Eigen::Transform<double,3,Eigen::Affine> getTransform_SwarmTOVehicleEKF() const
    {
        return m_swarmTOvehicleEKF;
    }

private:
    Eigen::Transform<double,3,Eigen::Affine> m_vehicleHomeTOswarm;
    Eigen::Transform<double,3,Eigen::Affine> m_vehicleEKFTOswarm;

    Eigen::Transform<double,3,Eigen::Affine> m_swarmTOvehicleHome;
    Eigen::Transform<double,3,Eigen::Affine> m_swarmTOvehicleEKF;

    bool transformToSwarmAltitude = true;
};

#endif // STATE_DATA_MAVLINK_H
