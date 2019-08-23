#ifndef MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H
#define MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"
#include "module_vehicle_MAVLINK/mavlink_coordinate_frames.h"

#include "data_generic_command_item/do_items/action_set_global_origin.h"

namespace MAVLINKUXVControllers {

using GPSOriginSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_SetGlobalOrigin>,
    MavlinkEntityKey,
    command_item::Action_SetGlobalOrigin,
    mavlink_set_gps_global_origin_t,
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
>;

using GPSOriginFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_SetGlobalOrigin>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_gps_global_origin_t,
    MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
>;

class Controller_SetGPSGlobalOrigin : public BasicMavlinkController_ModuleKeyed<command_item::Action_SetGlobalOrigin>,
        public GPSOriginSend,
        public GPSOriginFinish
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const command_item::Action_SetGlobalOrigin &commandItem, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_gps_global_origin_t &gpsOriginItem, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(commandItem.getTargetSystem());

        gpsOriginItem = initializeGlobalOrigin();
        gpsOriginItem.target_system = commandItem.getTargetSystem();

        FillGPSOriginItem(commandItem,gpsOriginItem);

        return true;
    }


    virtual bool Finish_Receive(const mavlink_mission_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.type; //this is MAV_MISSION_RESULT
        return true;
    }

protected:
    void FillGPSOriginItem(const command_item::Action_SetGlobalOrigin &commandItem, mavlink_set_gps_global_origin_t &mavlinkItem);

    mavlink_set_gps_global_origin_t initializeGlobalOrigin()
    {
        mavlink_set_gps_global_origin_t item;
        item.target_system = 0;
        item.latitude = 0.0;
        item.longitude = 0.0;
        item.altitude = 0.0;

        return item;
    }

public:
    Controller_SetGPSGlobalOrigin(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_SetGlobalOrigin>(cb, queue, linkChan),
        GPSOriginSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_gps_global_origin_t>(mavlink_msg_set_gps_global_origin_encode_chan)),
        GPSOriginFinish(this, mavlink_msg_gps_global_origin_decode)
    {

    }

    virtual ~Controller_SetGPSGlobalOrigin() = default;

};

} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_SET_GPS_GLOBAL_ORIGIN_H
