#ifndef HELPER_PREVIOUS_GUIDED_MAVLINK_H
#define HELPER_PREVIOUS_GUIDED_MAVLINK_H

#include "helper_previous_transmission_base_mace.h"

namespace DataInterface_MACE {

enum guidedItemEnum{
    WAYPOINT
};

template <class T>
class PreviousGuided : public PreviousTransmissionBase<guidedItemEnum>
{
public:
    PreviousGuided(const guidedItemEnum &objType, const T &data):
        PreviousTransmissionBase(objType), obj(data)
    {

    }

    PreviousGuided(const PreviousGuided &rhs)
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

#endif // HELPER_PREVIOUS_GUIDED_MAVLINK_H
