#ifndef HELPER_PREVIOUS_TRANSMISSION_BASE_MACE_H
#define HELPER_PREVIOUS_TRANSMISSION_BASE_MACE_H

namespace DataInterface_MACE {

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

} //end of namespace DataInterface_MACE

#endif // HELPER_PREVIOUS_TRANSMISSION_BASE_MACE_H
