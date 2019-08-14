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

private:
    void updatePositionalTransformations();

public:
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalPosition;
    Data::DataGetSetNotifier<mace::pose::CartesianPosition_3D> vehicleLocalPosition;
    Data::DataGetSetNotifier<mace::pose::Rotation_3D> vehicleAttitude;
    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleAirspeed;

public:
    Eigen::Transform<double,3,Eigen::Affine> getTransform_VehicleTOSwarm() const
    {
        return m_vehicleTOswarm;
    }

    Eigen::Transform<double,3,Eigen::Affine> getTransform_SwarmTOVehicle() const
    {
        return m_swarmTOvehicle;
    }
private:
    Eigen::Transform<double,3,Eigen::Affine> m_vehicleTOswarm;
    Eigen::Transform<double,3,Eigen::Affine> m_swarmTOvehicle;
};

#endif // STATE_DATA_MAVLINK_H
