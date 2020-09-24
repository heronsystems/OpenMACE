#include "state_data_mavlink.h"

StateData_MAVLINK::StateData_MAVLINK()
{
}

std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> StateData_MAVLINK::GetTopicData()
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;
    return rtnVector;
}
