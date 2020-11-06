#ifndef ENVIRONMENT_DATA_MAVLINK_H
#define ENVIRONMENT_DATA_MAVLINK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "data/data_get_set_notifier.h"

#include "base/pose/pose_components.h"
#include "base/measurements/base_speed.h"

#include "base_topic/base_topic_components.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"

class EnvironmentData_MAVLINK
{
public:
    EnvironmentData_MAVLINK();

    EnvironmentData_MAVLINK(const EnvironmentData_MAVLINK &copy) = delete;

    ~EnvironmentData_MAVLINK() = default;

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetTopicData();

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

#endif // ENVIRONMENT_DATA_MAVLINK_H
