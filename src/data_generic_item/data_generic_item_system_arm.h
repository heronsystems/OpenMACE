#ifndef DATA_GENERIC_ITEM_SYSTEM_ARM_H
#define DATA_GENERIC_ITEM_SYSTEM_ARM_H

#include "mace.h"

namespace DataGenericItem {

class DataGenericItem_SystemArm
{
public:
    DataGenericItem_SystemArm();

    DataGenericItem_SystemArm(const bool &arm);

    DataGenericItem_SystemArm(const DataGenericItem_SystemArm &copyObj);

    DataGenericItem_SystemArm(const mace_vehicle_armed_t &copyObj);

    void setSystemArm(const bool &arm)
    {
        this->armed = arm;
    }

    bool getSystemArm() const
    {
        return this->armed;
    }

    mace_vehicle_armed_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_SystemArm &rhs)
    {
        this->armed = rhs.armed;
    }

    bool operator == (const DataGenericItem_SystemArm &rhs) {
        if(this->armed != rhs.armed){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_SystemArm &rhs) {
        return !(*this == rhs);
    }


protected:
    bool armed = false;
};

} //end of namespace DataGenericItem



#endif // DATA_GENERIC_ITEM_SYSTEM_ARM_H
