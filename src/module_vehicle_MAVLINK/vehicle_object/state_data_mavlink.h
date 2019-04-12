#ifndef STATE_DATA_MAVLINK_H
#define STATE_DATA_MAVLINK_H

#include "data/data_get_set_notifier.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

//typedef void(*CallbackFunctionPtr_State)(void*, DataState::StateGlobalPosition&);

class StateData_MAVLINK
{
public:
    StateData_MAVLINK() = default;

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
    Data::DataGetSetNotifier<DataState::StateGlobalPosition> vehicleGlobalPosition;
    Data::DataGetSetNotifier<DataState::StateGlobalPositionEx> vehicleGlobalPositionEx;
    Data::DataGetSetNotifier<DataState::StateLocalPosition> vehicleLocalPosition;
    Data::DataGetSetNotifier<DataState::StateAttitude> vehicleAttitude;
    Data::DataGetSetNotifier<DataState::StateAirspeed> vehicleAirspeed;
};

#endif // STATE_DATA_MAVLINK_H
