#ifndef COMMAND_ITEM_H
#define COMMAND_ITEM_H

#include <string>

namespace DataInterface_MAVLINK{

enum commandItemEnum{
    COMMAND_INT,
    COMMAND_LONG,
    COMMAND_MODE
};

inline std::string getCommandItemEnumString(const commandItemEnum &value)
{
    std::string rtnValue;

    switch (value) {
    case COMMAND_INT:
        rtnValue = "type integer";
        break;
    case COMMAND_LONG:
        rtnValue = "type long";
        break;
    case COMMAND_MODE:
        rtnValue = "system mode";
        break;
    default:
        break;
    }

    return rtnValue;
}

} // end of namespace DataInterface_MAVLINK

#endif // COMMAND_ITEM_H
