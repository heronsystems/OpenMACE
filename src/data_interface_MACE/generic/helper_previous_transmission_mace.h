#ifndef HELPER_PREVIOUS_TRANSMISSION_MACE_H
#define HELPER_PREVIOUS_TRANSMISSION_MACE_H

#include "helper_previous_transmission_base_mace.h"

#include "mace_core/module_characteristics.h"
#include "common/optional_parameter.h"


namespace DataInterface_MACE {

enum commsItemEnum{
    ITEM_RXLIST,
    ITEM_RXGENLIST,
    ITEM_RXITEM,
    ITEM_RXHOME,
    ITEM_SETHOME,
    ITEM_TXCOUNT,
    ITEM_TXITEM
};

inline std::string getCommsItemEnumString(const commsItemEnum &type)
{
    std::string rtnValue;

    switch (type) {
    case ITEM_RXLIST:
        rtnValue = "requesting mission list";
        break;
    case ITEM_RXITEM:
        rtnValue = "requesting mission item";
        break;
    case ITEM_RXHOME:
        rtnValue = "requesting system home";
        break;
    case ITEM_TXCOUNT:
        rtnValue = "transmitting mission count";
        break;
    case ITEM_TXITEM:
        rtnValue = "transmitting mission item";
        break;
    default:
        break;
    }

    return rtnValue;
}

template <class T>
class PreviousTransmission : public PreviousTransmissionBase<commsItemEnum>
{
public:
    PreviousTransmission(const commsItemEnum &objType, const T &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>(), const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()):
        PreviousTransmissionBase(objType), obj(data), m_sender(sender), m_target(target)
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

    bool HasSender() const
    {
        return m_sender.IsSet();
    }

    MaceCore::ModuleCharacteristic Sender() const {
        return m_sender.Value();
    }

    MaceCore::ModuleCharacteristic Target() const {
        return m_target.Value();
    }

private:
    T obj;
    OptionalParameter<MaceCore::ModuleCharacteristic> m_sender;
    OptionalParameter<MaceCore::ModuleCharacteristic> m_target;
};

} //end of namespace DataInterface_MACE

#endif // HELPER_PREVIOUS_TRANSMISSION_MACE_H
