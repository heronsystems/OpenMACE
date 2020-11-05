#ifndef CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H
#define CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_broadcast.h"


#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/do_items/action_dynamic_target.h"

#include "data_generic_command_item/target_items/dynamic_target_orientation.h"
#include "base/pose/rotation_3D.h"

namespace MAVLINKUXVControllers {

using namespace mace::pose;

using GuidedTGTAttBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>,
    command_item::Action_DynamicTarget,
    mavlink_set_attitude_target_t
>;

class ControllerGuidedTargetItem_Attitude : public BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>,
        public GuidedTGTAttBroadcast
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

    mavlink_set_attitude_target_t m_targetMSG;

private:
    bool doesMatchTransmitted(const mavlink_position_target_global_int_t &msg) const;

protected:

    void Construct_Broadcast(const command_item::Action_DynamicTarget &commandItem, const MavlinkEntityKey &sender, mavlink_set_attitude_target_t &targetItem) override
    {
        UNUSED(sender);

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = commandItem.getTargetSystem();
        targetItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        command_target::DynamicTarget* currentTarget = commandItem.getDynamicTarget();

        if(currentTarget->getTargetType() == command_target::DynamicTarget::TargetTypes::ORIENTATION)
            FillTargetItem(*currentTarget->targetAs<command_target::DynamicTarget_Orientation>(),targetItem);

        m_targetMSG = targetItem;
    }

protected:
    void FillTargetItem(const command_target::DynamicTarget_Orientation &command, mavlink_set_attitude_target_t &mavlinkItem);

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
        targetItem.type_mask = (uint8_t)65535; //by default we want to ignore all of the values
        return targetItem;
    }

public:
    ControllerGuidedTargetItem_Attitude(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>(cb, queue, linkChan),
        GuidedTGTAttBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_attitude_target_t>(mavlink_msg_set_attitude_target_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_ATTITUDE_H
