#ifndef CALLBACK_INTERFACE_DATA_MAVLINK_H
#define CALLBACK_INTERFACE_DATA_MAVLINK_H

#include <memory>

#include "mavlink.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_item/data_generic_item_systemtime.h"

#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

namespace DataInterface_MAVLINK{


class CallbackInterface_DataMAVLINK
{
public:
    CallbackInterface_DataMAVLINK();

public:
    virtual void cbi_VehicleStateData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;
    virtual void cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;

    virtual void cbi_VehicleSystemTime(const int &systemID, std::shared_ptr<DataGenericItem::DataGenericItem_SystemTime> systemTime) = 0;

    virtual void cbi_VehicleHome(const int &systemID, const command_item::SpatialHome &home) = 0;
    virtual void cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList) = 0;
    virtual void cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current) = 0;

    virtual void cbi_VehicleCommandACK(const int &systemID, const mavlink_command_ack_t &cmdACK) = 0;
    virtual void cbi_VehicleMissionACK(const MissionItem::MissionACK &ack) = 0;
};

} //end of namespace DataInterface_MAVLINK
#endif // CALLBACK_INTERFACE_DATA_MAVLINK_H
