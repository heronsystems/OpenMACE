#include "data_generic_item_system_arm.h"

namespace DataGenericItem {

DataGenericItem_SystemArm::DataGenericItem_SystemArm() :
    armed(false)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const bool &arm) :
    armed(arm)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const DataGenericItem_SystemArm &copyObj)
{
    this->armed = copyObj.getSystemArm();
}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const mace_vehicle_armed_t &copyObj)
{
    if(copyObj.vehicle_armed == 1)
        this->armed = true;
    else
        this->armed = false;
}

mace_vehicle_armed_t DataGenericItem_SystemArm::getMACECommsObject() const
{
    mace_vehicle_armed_t rtnObj;

    rtnObj.vehicle_armed = this->armed ? 1 : 0;

    return rtnObj;
}

mace_message_t DataGenericItem_SystemArm::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_vehicle_armed_t armed = getMACECommsObject();
    mace_msg_vehicle_armed_encode_chan(systemID,compID,chan,&msg,&armed);
    return msg;
}

} //end of namespace DataGenericItem
