#include "status_data_mavlink.h"

StatusData_MAVLINK::StatusData_MAVLINK()
{
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> StatusData_MAVLINK::GetTopicData()
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

    return rtnVector;
}
