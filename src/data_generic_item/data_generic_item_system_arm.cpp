#include "data_generic_item_system_arm.h"

namespace DataGenericItem {

DataGenericItem_SystemArm::DataGenericItem_SystemArm() :
    _armed(false)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const bool &arm) :
    _armed(arm)
{

}

DataGenericItem_SystemArm::DataGenericItem_SystemArm(const DataGenericItem_SystemArm &copyObj)
{
    this->_armed = copyObj.getSystemArm();
}

} //end of namespace DataGenericItem
