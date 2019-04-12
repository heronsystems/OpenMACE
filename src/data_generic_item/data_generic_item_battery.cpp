#include "data_generic_item_battery.h"

namespace DataGenericItem {

DataGenericItem_Battery::DataGenericItem_Battery() :
    voltage(0.0), current(0.0), batteryRemaing(0.0)
{

}

DataGenericItem_Battery::DataGenericItem_Battery(const DataGenericItem_Battery &copyObj)
{
    this->voltage = copyObj.getBatteryVoltage();
    this->current = copyObj.getBatteryCurrent();
    this->batteryRemaing = copyObj.getBatteryRemaining();
}

DataGenericItem_Battery::DataGenericItem_Battery(const mace_battery_status_t &copyObj)
{
    this->voltage = copyObj.voltage_battery / 1000.0;
    this->current = copyObj.current_battery / 1000.0;
    this->batteryRemaing = copyObj.battery_remaining;
}

mace_battery_status_t DataGenericItem_Battery::getMACECommsObject() const
{
    mace_battery_status_t rtnObj;
    rtnObj.voltage_battery = (uint16_t)(this->voltage * 1000.0);
    rtnObj.current_battery = (int16_t)(this->current * 1000.0);
    rtnObj.battery_remaining = (int8_t)this->getBatteryRemaining();

    return rtnObj;
}

mace_message_t DataGenericItem_Battery::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_battery_status_t battery = getMACECommsObject();
    mace_msg_battery_status_encode_chan(systemID,compID,chan,&msg,&battery);
    return msg;
}

} //end of namespace DataGenericItem
