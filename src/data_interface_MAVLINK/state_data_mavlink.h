#ifndef STATE_DATA_MAVLINK_OLD_H
#define STATE_DATA_MAVLINK_OLD_H

#include "data/data_get_set_notifier.h"

#include "components/ardupilot_component_flight_mode.h"

#include "data_generic_item_topic/data_generic_item_topic_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

typedef void(*CallbackFunctionPtr_State)(void*, DataState::StateGlobalPosition&);

namespace DataInterface_MAVLINK {

class StateData_MAVLINK
{
public:
    StateData_MAVLINK();

    void connectCallback_State(CallbackFunctionPtr_State cb, void *p);

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> GetTopicData();

    void performCallback();

public:
    Data::DataGetSetNotifier<DataARDUPILOT::ARDUPILOTComponent_FlightMode> vehicleFlightMode;

public:
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Heartbeat> vehicleHeartbeat;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_FlightMode> vehicleMode;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_SystemArm> vehicleArm;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Battery> vehicleFuel;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_GPS> vehicleGPSStatus;
    Data::DataGetSetNotifier<DataGenericItem::DataGenericItem_Text> vehicleTextAlert;

public:
    Data::DataGetSetNotifier<DataState::StateGlobalPosition> vehicleGlobalPosition;
    Data::DataGetSetNotifier<DataState::StateGlobalPositionEx> vehicleGlobalPositionEx;
    Data::DataGetSetNotifier<DataState::StateLocalPosition> vehicleLocalPosition;
    Data::DataGetSetNotifier<DataState::StateAttitude> vehicleAttitude;
    Data::DataGetSetNotifier<DataState::StateAirspeed> vehicleAirspeed;

private:
    CallbackFunctionPtr_State m_CBCmdLng;
    void *m_p;

};

} //end of namespace DataInterface_MAVLINK
#endif // STATE_DATA_MAVLINK_OLD_H
