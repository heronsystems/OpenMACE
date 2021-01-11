#ifndef STATE_DATA_MAVLINK_H
#define STATE_DATA_MAVLINK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "data/data_get_set_notifier.h"

#include "base/pose/pose_components.h"
#include "base/measurements/base_speed.h"
#include "base/vehicle/vehicle_state.h"

#include "base_topic/base_topic_components.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"

class StateData_MAVLINK
{
public:
    StateData_MAVLINK(const std::string &ID = "");

    StateData_MAVLINK(const StateData_MAVLINK &copy) = delete;

    ~StateData_MAVLINK() = default;

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetTopicData();

public:
  std::string getAgentID() const;

private:
  void callbackUpdateVehicleState();

  Data::EnvironmentTime whenWasLastUpdate() const;

public:
    Data::DataGetSetNotifier<VehicleState_Geodetic3D> m_CompleteVehicleGeoState;
    Data::DataGetSetNotifier<VehicleState_Cartesian3D> m_CompleteVehicleCarState;

public:
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalPosition;
    Data::DataGetSetNotifier<mace::pose::CartesianPosition_3D> vehicleLocalPosition;

    Data::DataGetSetNotifier<mace::pose::Velocity_Cartesian3D> vehicleLocalVelocity;

    Data::DataGetSetNotifier<mace::pose::Rotation_3D> vehicleAttitude;
    Data::DataGetSetNotifier<mace::pose::Velocity_Rotation3D> vehicleRotationalVelocity;

    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleAirspeed;
    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleGroundSpeed;

private:
  std::string _agentID = "";
};

#endif // STATE_DATA_MAVLINK_H
