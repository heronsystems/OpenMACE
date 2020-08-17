#include "environment_data_mavlink.h"

EnvironmentData_MAVLINK::EnvironmentData_MAVLINK()
{
    m_swarmTOvehicleHome = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_swarmTOvehicleHome.translation() = Eigen::Vector3d::Zero();

    m_swarmTOvehicleEKF = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_swarmTOvehicleEKF.translation() = Eigen::Vector3d::Zero();

    m_vehicleHomeTOswarm = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_vehicleHomeTOswarm.translation() = Eigen::Vector3d::Zero();

    m_vehicleEKFTOswarm = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_vehicleEKFTOswarm.translation() = Eigen::Vector3d::Zero();

    vehicleGlobalOrigin.AddNotifier(this,[this]{
        updatePositionalTransformations_EKF();
    });

    vehicleGlobalHome.AddNotifier(this,[this]{
        updatePositionalTransformations_Home();
    });

    swarmGlobalOrigin.AddNotifier(this,[this]{
        updatePositionalTransformations_Home();
        updatePositionalTransformations_EKF();
    });

}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> EnvironmentData_MAVLINK::GetTopicData()
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    return rtnVector;
}

void EnvironmentData_MAVLINK::set_ShouldTransformLocalAltitude(const bool &transform)
{
    transformToSwarmAltitude = transform;
}

bool EnvironmentData_MAVLINK::shouldTransformLocalAltitude() const
{
    return transformToSwarmAltitude;
}

void EnvironmentData_MAVLINK::updatePositionalTransformations_Home()
{
    //check to see if both the origins have been set, if not, we have no knowledge
    //on how to achieve the correct transformation
    if(swarmGlobalOrigin.hasBeenSet() && vehicleGlobalHome.hasBeenSet())
    {
        mace::pose::GeodeticPosition_3D swarmOrigin = swarmGlobalOrigin.get();
        mace::pose::GeodeticPosition_3D vehicleHome = vehicleGlobalHome.get();

        double bearingTo = swarmOrigin.polarBearingTo(&vehicleHome);
        double translationalDistance = swarmOrigin.distanceBetween2D(&vehicleHome);
        double altitudeDifference = swarmOrigin.deltaAltitude(&vehicleHome);

        double distanceTranslateX = translationalDistance * cos(correctForAcuteAngle(bearingTo));
        double distanceTranslateY = translationalDistance * sin(correctForAcuteAngle(bearingTo));
        correctSignFromPolar(distanceTranslateX, distanceTranslateY, bearingTo);

        m_vehicleHomeTOswarm.translation() = Eigen::Vector3d(distanceTranslateY, distanceTranslateX, -altitudeDifference); //This will store it in NED
        m_swarmTOvehicleHome.translation() = m_vehicleHomeTOswarm.translation() * -1;
    }
}

void EnvironmentData_MAVLINK::updatePositionalTransformations_EKF()
{
    //check to see if both the origins have been set, if not, we have no knowledge
    //on how to achieve the correct transformation
    if(swarmGlobalOrigin.hasBeenSet() && vehicleGlobalOrigin.hasBeenSet())
    {
        mace::pose::GeodeticPosition_3D swarmOrigin = swarmGlobalOrigin.get();
        mace::pose::GeodeticPosition_3D vehicleEKF = vehicleGlobalOrigin.get();

        double bearingTo = swarmOrigin.polarBearingTo(&vehicleEKF);
        double translationalDistance = swarmOrigin.distanceBetween2D(&vehicleEKF);
        double altitudeDifference = swarmOrigin.deltaAltitude(&vehicleEKF);

        double distanceTranslateX = translationalDistance * cos(correctForAcuteAngle(bearingTo));
        double distanceTranslateY = translationalDistance * sin(correctForAcuteAngle(bearingTo));
        correctSignFromPolar(distanceTranslateX, distanceTranslateY, bearingTo);

        m_vehicleEKFTOswarm.translation() = Eigen::Vector3d(distanceTranslateY, distanceTranslateX, altitudeDifference);
        m_swarmTOvehicleEKF.translation() = m_vehicleEKFTOswarm.translation() * -1;
    }
}
