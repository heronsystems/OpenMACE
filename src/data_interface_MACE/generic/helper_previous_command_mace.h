#ifndef HELPER_PREVIOUS_COMMAND_MACE_H
#define HELPER_PREVIOUS_COMMAND_MACE_H

#include <string>

#include "helper_previous_transmission_base_mace.h"

namespace DataInterface_MACE {

enum commandItemEnum{
    COMMAND_SHORT,
    COMMAND_LONG,
    COMMAND_MODE
};

inline std::string getCommandItemEnumString(const commandItemEnum &type)
{
    std::string rtnValue;

    switch (type) {
    case COMMAND_SHORT:
        rtnValue = "command short";
        break;
    case COMMAND_LONG:
        rtnValue = "command long";
        break;
    case COMMAND_MODE:
        rtnValue = "command mode";
        break;
    default:
        break;
    }

    return rtnValue;
}

template <class T>
class PreviousCommand : public PreviousTransmissionBase<commandItemEnum>
{
public:
    PreviousCommand(const commandItemEnum &objType, const T &data):
        PreviousTransmissionBase(objType), obj(data)
    {

    }

    PreviousCommand(const PreviousCommand &rhs)
    {
        this->obj = rhs.obj;
        this->type = rhs.type;
    }

    void setData(const T &data)
    {
        this->obj = data;
    }

    T getData() const
    {
        return this->obj;
    }
private:
    T obj;
};

} //end of namespace DataInterfaceMACE

#endif // HELPER_PREVIOUS_COMMAND_MAVLINK_H
