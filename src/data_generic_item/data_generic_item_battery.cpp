#include "data_generic_item_battery.h"

namespace DataGenericItem {

DataGenericItem_Battery::DataGenericItem_Battery() :
    _voltage(0.0), _current(0.0), _batteryRemaing(0.0)
{

}

DataGenericItem_Battery::DataGenericItem_Battery(const DataGenericItem_Battery &copyObj)
{
    this->_voltage = copyObj.getBatteryVoltage();
    this->_current = copyObj.getBatteryCurrent();
    this->_batteryRemaing = copyObj.getBatteryRemaining();
}

DataGenericItem_Battery::DataGenericItem_Battery(const mavlink_battery_status_t &copyObj)
{

    this->_voltage = copyObj.voltages[0] / 1000.0;
    this->_current = copyObj.current_battery / 1000.0;
    this->_batteryRemaing = copyObj.battery_remaining;
}

mavlink_battery_status_t DataGenericItem_Battery::getMACECommsObject() const
{
    mavlink_battery_status_t rtnObj;
    rtnObj.voltages[0] = static_cast<uint16_t>((this->_voltage * 1000.0));
    rtnObj.current_battery = static_cast<int16_t>(this->_current * 1000.0);
    rtnObj.battery_remaining = static_cast<int8_t>(this->getBatteryRemaining());

    return rtnObj;
}

mavlink_message_t DataGenericItem_Battery::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_battery_status_t battery = getMACECommsObject();
    mavlink_msg_battery_status_encode_chan(systemID,compID,chan,&msg,&battery);
    return msg;
}

} //end of namespace DataGenericItem
