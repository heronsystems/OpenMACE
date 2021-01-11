#include "command_system_mode.h"

namespace ExternalLink {

ControllerCommand_SystemMode::ControllerCommand_SystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
    Controller_GenericShortCommand<command_item::ActionChangeMode, static_cast<uint8_t>(MAV_CMD::MAV_CMD_DO_SET_MODE)>(cb, queue, linkChan)
{

}


void ControllerCommand_SystemMode::FillCommand(const command_item::ActionChangeMode &commandItem, mavlink_command_short_t &cmd) const
{
UNUSED(commandItem);
UNUSED(cmd);
//        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
//        {
//            cmd.param2 = commandItem.position->getPosXFlag();
//            cmd.param3 = commandItem.position->getPosYFlag();
//            cmd.param4 = commandItem.position->getPosZFlag();
//            cmd.param5 = commandItem.position->getX() * pow(10,7);
//            cmd.param6 = commandItem.position->getY() * pow(10,7);
//            cmd.param7 = commandItem.position->getZ() * 1000;
//        }
}

void ControllerCommand_SystemMode::BuildCommand(const mavlink_command_short_t &message, command_item::ActionChangeMode &data) const
{
UNUSED(message);
UNUSED(data);
}


/*

    bool ControllerSystemMode::Construct_Send(const command_item::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_system_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        if(m_ActiveTransmits.find(target) != m_ActiveTransmits.cend() && m_ActiveTransmits.at(target) == true)
        {
            printf("System Mode is already being changed for this target, ignoring");
            return false;
        }

        m_ActiveTransmits.insert({target, true});

        for(size_t i = 0 ; i < sizeof(cmd.mode)/sizeof(*cmd.mode) ; i++)
        {
            cmd.mode[i] = 0;
        }

        cmd.target_system = commandItem.getTargetSystem();
        strcpy(cmd.mode, commandItem.getRequestMode().c_str());

        return true;
    }


    bool ControllerSystemMode::Construct_FinalObjectAndResponse(const mace_command_system_mode_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_system_mode_ack_t &ack, MaceCore::ModuleCharacteristic &dataKey, command_item::ActionChangeMode &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj = this->GetKeyFromSecondaryID(msg.target_system);

        queueObj = vehicleObj;

        dataKey = vehicleObj;
        data.setTargetSystem(msg.target_system);
        data.setRequestMode(std::string(msg.mode));

        ack.result = static_cast<uint8_t>(MAV_CMD_ACK::CA_RECEIVED);
        return true;
    }

    bool ControllerSystemMode::Finish_Receive(const mace_system_mode_ack_t &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        m_ActiveTransmits.erase(sender);
        return true;
    }

    ControllerSystemMode::ControllerSystemMode(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue * queue, int linkChan) :
        CONTROLLER_SYSTEMMODE_TYPE(cb, queue, linkChan),
        SystemModeSend(this, ModuleToSysIDCompIDConverter<mace_command_system_mode_t>(mavlink_msg_command_system_mode_encode_chan)),
        SystemMode_FinalReceiveRespond(this, mavlink_msg_command_system_mode_decode, ModuleToSysIDCompIDConverter<mace_system_mode_ack_t>(mavlink_msg_system_mode_ack_encode_chan)),
        SystemModeFinish(this, mavlink_msg_system_mode_ack_decode)
    {

    }
*/
}
