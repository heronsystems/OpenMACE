#ifndef CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
#define CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H

#include <mavlink.h>

#include "common/common.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"
#include "controllers/actions/action_broadcast.h"


#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

#include "module_vehicle_MAVLINK/controllers/common.h"

#include "data_generic_command_item/do_items/action_dynamic_target.h"

#include "data_generic_command_item/target_items/dynamic_target_kinematic.h"


namespace MAVLINKUXVControllers {

using namespace mace::pose;

using GuidedTGTGlobalBroadcast = Controllers::ActionBroadcast<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>,
    command_item::Action_DynamicTarget,
    mavlink_set_position_target_global_int_t
>;


class ControllerGuidedTargetItem_Global : public BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>,
        public GuidedTGTGlobalBroadcast
{
private:

    std::unordered_map<MaceCore::ModuleCharacteristic, MaceCore::ModuleCharacteristic> m_CommandRequestedFrom;

    mavlink_set_position_target_global_int_t m_targetMSG;

private:
    bool doesMatchTransmitted(const mavlink_position_target_global_int_t &msg) const;

protected:

    void Construct_Broadcast(const command_item::Action_DynamicTarget &commandItem, const MavlinkEntityKey &sender, mavlink_set_position_target_global_int_t &targetItem) override
    {
        UNUSED(sender);

        targetItem = initializeMAVLINKTargetItem();
        targetItem.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());
        targetItem.target_component = static_cast<uint8_t>(MaceCore::ModuleClasses::VEHICLE_COMMS);

        command_target::DynamicTarget* currentTarget = commandItem.getDynamicTarget();

        if(currentTarget->getTargetType() == command_target::DynamicTarget::TargetTypes::KINEMATIC)
            FillTargetItem(*currentTarget->targetAs<command_target::DynamicTarget_Kinematic>(),targetItem);

        m_targetMSG = targetItem;
    }

protected:
    void FillTargetItem(const command_target::DynamicTarget_Kinematic &command, mavlink_set_position_target_global_int_t &mavlinkItem);

    mavlink_set_position_target_global_int_t initializeMAVLINKTargetItem()
    {
        mavlink_set_position_target_global_int_t targetItem;
        targetItem.afx = 0.0;
        targetItem.afy = 0.0;
        targetItem.afz = 0.0;
        targetItem.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        targetItem.target_component =0;
        targetItem.target_system = 0;
        targetItem.time_boot_ms = 0;
        targetItem.type_mask = 65535; //by default we want to ignore all of the values
        targetItem.vx = 0.0;
        targetItem.vy = 0.0;
        targetItem.vz = 0.0;
        targetItem.lat_int = 0;
        targetItem.lon_int = 0;
        targetItem.yaw_rate = 0.0;
        targetItem.alt = 0.0;

        return targetItem;
    }

public:
    ControllerGuidedTargetItem_Global(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<command_item::Action_DynamicTarget>(cb, queue, linkChan),
        GuidedTGTGlobalBroadcast(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_position_target_global_int_t>(mavlink_msg_set_position_target_global_int_encode_chan))
    {

    }

private:


};

} //end of namespace MAVLINKVehicleControllers


#endif // CONTROLLER_GUIDED_TARGET_ITEM_GLOBAL_H
