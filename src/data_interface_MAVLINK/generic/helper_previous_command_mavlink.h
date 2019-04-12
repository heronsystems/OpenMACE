#ifndef HELPER_PREVIOUS_COMMAND_MAVLINK_H
#define HELPER_PREVIOUS_COMMAND_MAVLINK_H

#include "data_interface_MAVLINK/generic/command_item.h"
#include "data_interface_MAVLINK/generic/helper_previous_transmission.h"

namespace DataInterface_MAVLINK {

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

} //end of namespace DataInterface_MAVLINK

#endif // HELPER_PREVIOUS_COMMAND_MAVLINK_H
