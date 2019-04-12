#include "controller_system_mode.h"

#include "data/command_ack_type.h"

namespace ExternalLink {



    bool ControllerSystemMode::Construct_Send(const CommandItem::ActionChangeMode &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_command_system_mode_t &cmd, MaceCore::ModuleCharacteristic &queueObj)
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


    bool ControllerSystemMode::Construct_FinalObjectAndResponse(const mace_command_system_mode_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_system_mode_ack_t &ack, MaceCore::ModuleCharacteristic &dataKey, CommandItem::ActionChangeMode &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj = this->GetKeyFromSecondaryID(msg.target_system);

        queueObj = vehicleObj;

        dataKey = vehicleObj;
        data.setTargetSystem(msg.target_system);
        data.setRequestMode(std::string(msg.mode));

        ack.result = (uint8_t)Data::CommandACKType::CA_RECEIVED;
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

    ControllerSystemMode::ControllerSystemMode(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue * queue, int linkChan) :
        CONTROLLER_SYSTEMMODE_TYPE(cb, queue, linkChan),
        SystemModeSend(this, ModuleToSysIDCompIDConverter<mace_command_system_mode_t>(mace_msg_command_system_mode_encode_chan)),
        SystemMode_FinalReceiveRespond(this, mace_msg_command_system_mode_decode, ModuleToSysIDCompIDConverter<mace_system_mode_ack_t>(mace_msg_system_mode_ack_encode_chan)),
        SystemModeFinish(this, mace_msg_system_mode_ack_decode)
    {

    }

}
