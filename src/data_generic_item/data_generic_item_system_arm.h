#ifndef DATA_GENERIC_ITEM_SYSTEM_ARM_H
#define DATA_GENERIC_ITEM_SYSTEM_ARM_H

namespace DataGenericItem {

class DataGenericItem_SystemArm
{
public:
    DataGenericItem_SystemArm();

    DataGenericItem_SystemArm(const bool &arm);

    DataGenericItem_SystemArm(const DataGenericItem_SystemArm &copyObj);

    void setSystemArm(const bool &arm)
    {
        this->_armed = arm;
    }

    bool getSystemArm() const
    {
        return this->_armed;
    }

public:
    void operator = (const DataGenericItem_SystemArm &rhs)
    {
        this->_armed = rhs._armed;
    }

    bool operator == (const DataGenericItem_SystemArm &rhs) {
        if(this->_armed != rhs._armed){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_SystemArm &rhs) {
        return !(*this == rhs);
    }


protected:
    bool _armed = false;
};

} //end of namespace DataGenericItem



#endif // DATA_GENERIC_ITEM_SYSTEM_ARM_H
