#include "data_generic_item_timesync.h"

namespace DataGenericItem {

DataGenericItem_Timesync::DataGenericItem_Timesync() :
    m_tc1(0), m_ts1(0)
{

}

DataGenericItem_Timesync::DataGenericItem_Timesync(const DataGenericItem_Timesync &copyObj)
{
    this->m_tc1 = copyObj.getTC1();
    this->m_ts1 = copyObj.getTS1();
}

DataGenericItem_Timesync::DataGenericItem_Timesync(const mavlink_timesync_t &copyObj)
{
    this->m_tc1 = copyObj.tc1;
    this->m_ts1 = copyObj.ts1;
}

mavlink_timesync_t DataGenericItem_Timesync::getMACECommsObject() const
{
    mavlink_timesync_t rtnObj;
    rtnObj.tc1 = this->m_tc1;
    rtnObj.ts1 = this->m_ts1;

    return rtnObj;
}

mavlink_message_t DataGenericItem_Timesync::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    UNUSED(chan);

    mavlink_message_t msg;
    mavlink_timesync_t timesync = getMACECommsObject();
    mavlink_msg_timesync_encode(systemID, compID, &msg, &timesync);
    return msg;
}

} //end of namespace DataGenericItem


