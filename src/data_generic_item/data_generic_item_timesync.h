#ifndef DATA_GENERIC_ITEM_TIMESYNC_H
#define DATA_GENERIC_ITEM_TIMESYNC_H

#include <iostream>

#include <mavlink.h>

#include "common/common.h"

namespace DataGenericItem {

class DataGenericItem_Timesync
{
public:
    DataGenericItem_Timesync();

    DataGenericItem_Timesync(const DataGenericItem_Timesync &copyObj);

    DataGenericItem_Timesync(const mavlink_timesync_t &copyObj);



    void setTC1(const uint64_t &tc1) {
        this->m_tc1 = tc1;
    }

    uint64_t getTC1() const {
        return m_tc1;
    }

    void setTS1(const uint64_t &ts1) {
        this->m_ts1 = ts1;
    }

    uint64_t getTS1() const {
        return m_ts1;
    }


    mavlink_timesync_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_Timesync &rhs)
    {
        this->m_tc1 = rhs.m_tc1;
        this->m_ts1= rhs.m_ts1;
    }

    bool operator == (const DataGenericItem_Timesync &rhs) {
        if(this->m_tc1 != rhs.m_tc1){
            return false;
        }
        if(this->m_ts1 != rhs.m_ts1){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Timesync &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out << "Vehicle Timesync ( tc1: " << m_tc1 << ", ts1: " << m_ts1 << ")";
        return out;
    }

protected:
    uint64_t m_tc1;
    uint64_t m_ts1;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_SYSTEMTIME_H
