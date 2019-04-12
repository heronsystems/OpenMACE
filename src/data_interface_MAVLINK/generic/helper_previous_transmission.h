#ifndef HELPER_PREVIOUS_TRANSMISSION_H
#define HELPER_PREVIOUS_TRANSMISSION_H

#include "comms_item.h"

namespace DataInterface_MAVLINK {

template <class Base>
class PreviousTransmissionBase
{
public:

    PreviousTransmissionBase(const Base &objType):
        type(objType)
    {

    }

    PreviousTransmissionBase(const PreviousTransmissionBase &rhs)
    {
        this->type = rhs.type;
    }

    void setType(const Base &objType)
    {
        this->type = objType;
    }

    Base getType() const
    {
        return this->type;
    }
private:
    Base type;
};

template <class T>
class PreviousTransmission : public PreviousTransmissionBase<commsItemEnum>
{
public:
    PreviousTransmission(const commsItemEnum &objType, const T &data):
        PreviousTransmissionBase(objType), obj(data)
    {

    }

    PreviousTransmission(const PreviousTransmission &rhs)
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

#endif // HELPER_PREVIOUS_TRANSMISSION_H
