#ifndef MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
#define MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H

#include "mavlink.h"

#include "common/common.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_finish.h"

#include "controllers/controllers_MAVLINK/common.h"


namespace MAVLINKUXVControllers {

struct MAVLINKModeStruct
{
    uint8_t targetID;
    uint8_t vehicleMode;
};

using SystemModeSend = Controllers::ActionSend<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<MAVLINKModeStruct>,
    MavlinkEntityKey,
    MAVLINKModeStruct,
    mavlink_set_mode_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

using SystemModeFinish = Controllers::ActionFinish<
    mavlink_message_t,
    MavlinkEntityKey,
    BasicMavlinkController_ModuleKeyed<MAVLINKModeStruct>,
    MavlinkEntityKey,
    uint8_t,
    mavlink_command_ack_t,
    MAVLINK_MSG_ID_COMMAND_ACK
>;

class ControllerSystemMode : public BasicMavlinkController_ModuleKeyed<MAVLINKModeStruct>,
        public SystemModeSend,
        public SystemModeFinish
{
private:

    std::unordered_map<int, int> m_CommandRequestedFrom;

protected:

    virtual bool Construct_Send(const MAVLINKModeStruct &commandMode, const MavlinkEntityKey &sender, const MavlinkEntityKey &target, mavlink_set_mode_t &mode, MavlinkEntityKey &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        mode.target_system = commandMode.targetID;
        mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode.custom_mode = commandMode.vehicleMode;

        return true;
    }


    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }

public:

    ControllerSystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        BasicMavlinkController_ModuleKeyed<MAVLINKModeStruct>(cb, queue, linkChan),
        SystemModeSend(this, MavlinkEntityKeyToSysIDCompIDConverter<mavlink_set_mode_t>(mavlink_msg_set_mode_encode_chan)),
        SystemModeFinish(this, mavlink_msg_command_ack_decode)
    {

    }

    virtual ~ControllerSystemMode() = default;


};

} //end of namespace MAVLINKVehicleControllers

#endif // MODULE_VEHICLE_MAVLINK_CONTROLLER_SYSTEM_MODE_H
