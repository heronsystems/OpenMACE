#include "data_generic_item_systemtime.h"

namespace DataGenericItem {

DataGenericItem_SystemTime::DataGenericItem_SystemTime() :
    usec_since_epoch(0), ms_since_system_boot(0)
{

}

DataGenericItem_SystemTime::DataGenericItem_SystemTime(const DataGenericItem_SystemTime &copyObj)
{
    this->usec_since_epoch = copyObj.getUsecSinceEpoch();
    this->ms_since_system_boot = copyObj.getMillisecondsSinceBoot();
}

DataGenericItem_SystemTime::DataGenericItem_SystemTime(const mace_system_time_t &copyObj)
{
    this->usec_since_epoch = copyObj.time_unix_usec;
    this->ms_since_system_boot = copyObj.time_boot_ms;
}

mace_system_time_t DataGenericItem_SystemTime::getMACECommsObject() const
{
    mace_system_time_t rtnObj;
    rtnObj.time_unix_usec = this->usec_since_epoch;
    rtnObj.time_boot_ms = this->ms_since_system_boot;

    return rtnObj;
}

mace_message_t DataGenericItem_SystemTime::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    UNUSED(chan);

    mace_message_t msg;
    mace_system_time_t systemTime = getMACECommsObject();
    mace_msg_system_time_encode(systemID, compID, &msg, &systemTime);
    return msg;
}

} //end of namespace DataGenericItem


