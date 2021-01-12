#ifndef DATA_GENERIC_ITEM_SYSTEMTIME_H
#define DATA_GENERIC_ITEM_SYSTEMTIME_H

#include <iostream>

#include <mavlink.h>

#include "common/common.h"

namespace DataGenericItem {

class DataGenericItem_SystemTime
{
public:
    DataGenericItem_SystemTime();

    DataGenericItem_SystemTime(const DataGenericItem_SystemTime &copyObj);

    DataGenericItem_SystemTime(const mavlink_system_time_t &copyObj);

    void setTimeSinceEpoch(const uint64_t &usec) {
        this->usec_since_epoch = usec;
    }

    uint64_t getUsecSinceEpoch() const {
        return usec_since_epoch;
    }

    void setTimeSinceBoot(const uint32_t &ms) {
        this->ms_since_system_boot = ms;
    }

    uint32_t getMillisecondsSinceBoot() const {
        return ms_since_system_boot;
    }

    mavlink_system_time_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_SystemTime &rhs)
    {
        this->usec_since_epoch = rhs.usec_since_epoch;
        this->ms_since_system_boot = rhs.ms_since_system_boot;
    }

    bool operator == (const DataGenericItem_SystemTime &rhs) {
        if(this->usec_since_epoch != rhs.usec_since_epoch){
            return false;
        }
        if(this->ms_since_system_boot != rhs.ms_since_system_boot){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_SystemTime &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out << "Vehicle System Time ( usec_since_epoch: " << usec_since_epoch<< ", ms_since_system_boot: " << ms_since_system_boot<< ")";
        return out;
    }

protected:
    uint64_t usec_since_epoch;
    uint32_t ms_since_system_boot;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_SYSTEMTIME_H
