#include "command_land.h"

namespace ExternalLink {

    CommandLand::CommandLand(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialLand, (uint8_t)command_item::COMMANDTYPE::CI_NAV_LAND>(cb, queue, linkChan)
    {

    }


    void CommandLand::FillCommand(const command_item::SpatialLand &commandItem, mace_command_long_t &cmd) const
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

    void CommandLand::BuildCommand(const mace_command_long_t &message, command_item::SpatialLand &data) const
    {
        UNUSED(message);
        UNUSED(data);

//        data.setTargetSystem(-1);

//        if(message.param2 == 1)
//        {
//            data.position->setX(message.param5);
//        }
//        if(message.param3 == 1)
//        {
//            data.position->setY(message.param6);
//        }
//        if(message.param4 == 1)
//        {
//            data.position->setZ(message.param7);
//        }
    }

}
