#ifndef CONTROLLER_GUIDED_MISSION_ITEM_H
#define CONTROLLER_GUIDED_MISSION_ITEM_H

#include <mavlink.h>

#include "common/common.h"

#include "data_generic_command_item/spatial_items/spatial_waypoint.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "controllers/controllers_MAVLINK/common.h"

namespace MAVLINKUXVControllers {

template <typename T>
using GuidedMISend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MavlinkEntityKey,
    T,
    mavlink_mission_item_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;

template <typename T>
using GuidedMIFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_mission_ack_t,
    MAVLINK_MSG_ID_MISSION_ACK
>;

template <typename MISSIONITEM>
class ControllerGuidedMissionItem : public BasicMavlinkController_ModuleKeyed<MISSIONITEM>,
        public GuidedMISend<MISSIONITEM>,
        public GuidedMIFinish<MISSIONITEM>
{
private:

    std::unordered_map<MavlinkEntityKey, MavlinkEntityKey> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const MISSIONITEM &commandItem, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_mission_item_t &missionItem, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        queueObj = this->GetKeyFromSecondaryID(commandItem.getTargetSystem());

        missionItem = initializeMAVLINKMissionItem();
        missionItem.target_system = commandItem.getTargetSystem();
        missionItem.target_component = 0;

        FillMissionItem(commandItem,missionItem);

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
    void FillMissionItem(const command_item::SpatialWaypoint &commandItem, mavlink_mission_item_t &mavlinkItem);

    mavlink_mission_item_t initializeMAVLINKMissionItem()
    {
        mavlink_mission_item_t missionItem;
        missionItem.autocontinue = 1;
        missionItem.command = MAV_CMD_NAV_WAYPOINT;
        missionItem.current = 2;
        missionItem.frame = MAV_FRAME_GLOBAL;
        missionItem.param1 = 0.0;
        missionItem.param2 = 0.0;
        missionItem.param3 = 0.0;
        missionItem.param4 = 0.0;
        missionItem.seq = 0;
        missionItem.target_system = 0;
        missionItem.target_component = 0;
        missionItem.x = 0.0;
        missionItem.y = 0.0;
        missionItem.z = 0.0;
        missionItem.mission_type = 0;

        return missionItem;
    }

public:
    ControllerGuidedMissionItem(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<MISSIONITEM>(cb, queue, linkChan, "GuidedMissionItem"),
        GuidedMISend<MISSIONITEM>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_mission_item_t>(mavlink_msg_mission_item_encode_chan)),
        GuidedMIFinish<MISSIONITEM>(this, mavlink_msg_mission_ack_decode)
    {

    }

    virtual ~ControllerGuidedMissionItem() = default;

};

} //end of namespace MAVLINKVehicleControllers

#endif // CONTROLLER_GUIDED_MISSION_ITEM_H
