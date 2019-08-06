#include "state_data_mavlink.h"


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
