#include "command_item_ack.h"

namespace command_item {
CommandItemACK::CommandItemACK():
    cmd(COMMANDTYPE::CI_UNKNOWN),code(Data::CommandACKType::CA_UNKNOWN),originatingSystem(0),targetSystem(0)
{

}

CommandItemACK::CommandItemACK(const COMMANDTYPE &cmdType, const Data::CommandACKType &codeType):
    cmd(cmdType),code(codeType),originatingSystem(0),targetSystem(0)
{

}

CommandItemACK::CommandItemACK(const int &systemOrigin, const COMMANDTYPE &cmdType, const Data::CommandACKType &codeType, const int &systemTarget):
    cmd(cmdType),code(codeType),originatingSystem(systemOrigin),targetSystem(systemTarget)

{

}
}
