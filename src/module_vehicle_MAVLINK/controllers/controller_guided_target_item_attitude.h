#ifndef CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H
#define CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_broadcast.h"


#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"


namespace MAVLINKUXVControllers {

using namespace mace::pose;

template <typename T>
using GuidedTGTAttBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<T>,
    T,
    mavlink_set_position_target_global_int_t
>;

template <typename COMMANDDATASTRUCTURE>
class ControllerGuidedTargetItem_Attitude : public BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>,
        public GuidedTGTAttBroadcast<COMMANDDATASTRUCTURE>
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

    mavlink_set_position_target_global_int_t m_targetMSG;

private:
    bool doesMatchTransmitted(const mavlink_position_target_global_int_t &msg) const;

protected:

    void Construct_Broadcast(const COMMANDDATASTRUCTURE &commandItem, const MavlinkEntityKey &sender, mavlink_set_position_target_global_int_t &targetItem) override
    {
        UNUSED(sender);

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.getTargetSystem();
        targetItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        FillTargetItem(commandItem,targetItem);

        m_targetMSG = targetItem;
    }

protected:
    void FillTargetItem(const COMMANDDATASTRUCTURE &command, mavlink_set_attitude_target_t &mavlinkItem);

    mavlink_set_attitude_target_t initializeMAVLINKTargetItem()
    {
        mavlink_set_attitude_target_t targetItem;
        targetItem.body_roll_rate = 0.0;
        targetItem.body_pitch_rate = 0.0;
        targetItem.body_yaw_rate = 0.0;
        targetItem.thrust = 0.5;

        targetItem.q[0] = 1.0;
        targetItem.q[1] = 0.0;
        targetItem.q[2] = 0.0;
        targetItem.q[3] = 0.0;

        targetItem.target_component =0;
        targetItem.target_system = 0;
        targetItem.type_mask = 65535; //by default we want to ignore all of the values
        return targetItem;
    }

public:
    ControllerGuidedTargetItem_Attitude(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<COMMANDDATASTRUCTURE>(cb, queue, linkChan),
        GuidedTGTAttBroadcast<COMMANDDATASTRUCTURE>(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_attitude_target_t>(mavlink_msg_set_attitude_target_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H
