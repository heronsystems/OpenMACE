#include "command_item_ack.h"

namespace command_item {
CommandItemACK::CommandItemACK():
    cmd(MAV_CMD::MAV_CMD_ENUM_END),code(MAV_CMD_ACK::MAV_CMD_ACK_ENUM_END),originatingSystem(0),targetSystem(0)
{

}

CommandItemACK::CommandItemACK(const MAV_CMD &cmdType, const  MAV_CMD_ACK &codeType):
    cmd(cmdType),code(codeType),originatingSystem(0),targetSystem(0)
{

}

CommandItemACK::CommandItemACK(const int &systemOrigin, const MAV_CMD &cmdType, const MAV_CMD_ACK &codeType, const int &systemTarget):
    cmd(cmdType),code(codeType),originatingSystem(systemOrigin),targetSystem(systemTarget)

{

}
}
