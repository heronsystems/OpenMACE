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
    Data::DataGetSetNotifier<mace::pose::GeodeticPosition_3D> vehicleGlobalPosition;
    Data::DataGetSetNotifier<mace::pose::CartesianPosition_3D> vehicleLocalPosition;

    Data::DataGetSetNotifier<mace::pose::Velocity_Cartesian3D> vehicleLocalVelocity;

    Data::DataGetSetNotifier<mace::pose::Rotation_3D> vehicleAttitude;
    Data::DataGetSetNotifier<mace::pose::Velocity_Rotation3D> vehicleRotationalVelocity;

    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleAirspeed;
    Data::DataGetSetNotifier<mace::measurements::Speed> vehicleGroundSpeed;
};

#endif // STATE_DATA_MAVLINK_H
