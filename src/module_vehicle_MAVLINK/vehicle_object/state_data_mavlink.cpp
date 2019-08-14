#include "state_data_mavlink.h"

StateData_MAVLINK::StateData_MAVLINK()
{
    m_swarmTOvehicle = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_swarmTOvehicle.translation() = Eigen::Vector3d::Zero();

    m_vehicleTOswarm = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    m_swarmTOvehicle.translation() = Eigen::Vector3d::Zero();

    vehicleGlobalOrigin.AddNotifier(this,[this]{
        updatePositionalTransformations();
    });

    swarmGlobalOrigin.AddNotifier(this,[this]{
        updatePositionalTransformations();
    });
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> StateData_MAVLINK::GetTopicData()
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(vehicleHeartbeat.get());
    rtnVector.push_back(ptrHeartbeat);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrSystemMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(vehicleMode.get());
    rtnVector.push_back(ptrSystemMode);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(vehicleArm.get());
    rtnVector.push_back(ptrArm);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> ptrFuel = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>(vehicleFuel.get());
    rtnVector.push_back(ptrFuel);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> ptrGPSStatus = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>(vehicleGPSStatus.get());
    rtnVector.push_back(ptrGPSStatus);

//    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> ptrGlobalPosition = std::make_shared<DataStateTopic::StateGlobalPositionTopic>(vehicleGlobalPosition.get());
//    rtnVector.push_back(ptrGlobalPosition);
//    std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> ptrGlobalPositionEx = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>(vehicleGlobalPositionEx.get());
//    rtnVector.push_back(ptrGlobalPositionEx);
//    std::shared_ptr<DataStateTopic::StateLocalPositionTopic> ptrLocalPosition = std::make_shared<DataStateTopic::StateLocalPositionTopic>(vehicleLocalPosition.get());
//    rtnVector.push_back(ptrLocalPosition);
//    std::shared_ptr<DataStateTopic::StateAttitudeTopic> ptrAttitude = std::make_shared<DataStateTopic::StateAttitudeTopic>(vehicleAttitude.get());
//    rtnVector.push_back(ptrAttitude);
//    std::shared_ptr<DataStateTopic::StateAirspeedTopic> ptrAirspeed = std::make_shared<DataStateTopic::StateAirspeedTopic>(vehicleAirspeed.get());
//    rtnVector.push_back(ptrAirspeed);

    return rtnVector;
}


void StateData_MAVLINK::updatePositionalTransformations()
{
    //check to see if both the origins have been set, if not, we have no knowledge
    //on how to achieve the correct transformation
    if(swarmGlobalOrigin.hasBeenSet() && vehicleGlobalOrigin.hasBeenSet())
    {
        mace::pose::GeodeticPosition_3D swarmOrigin = swarmGlobalOrigin.get();
        mace::pose::GeodeticPosition_3D vehicleOrigin = vehicleGlobalOrigin.get();

        double bearingTo = swarmOrigin.polarBearingTo(&vehicleOrigin);
        double translationalDistance = swarmOrigin.distanceBetween2D(&vehicleOrigin);
        double altitudeDifference = swarmOrigin.deltaAltitude(&vehicleOrigin);

        double distanceTranslateX = translationalDistance * cos(correctForAcuteAngle(bearingTo));
        double distanceTranslateY = translationalDistance * sin(correctForAcuteAngle(bearingTo));
        correctSignFromPolar(distanceTranslateX, distanceTranslateY, bearingTo);

        m_vehicleTOswarm.translation() = Eigen::Vector3d(distanceTranslateX, distanceTranslateY, altitudeDifference);
        m_swarmTOvehicle.translation() = m_vehicleTOswarm.translation() * -1;
    }
}
