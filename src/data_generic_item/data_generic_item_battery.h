#ifndef DATA_GENERIC_ITEM_FUEL_H
#define DATA_GENERIC_ITEM_FUEL_H

#include <iostream>
#include <math.h>
#include <limits>

#include "mavlink.h"

namespace DataGenericItem {

class DataGenericItem_Battery
{
public:
    DataGenericItem_Battery();

    DataGenericItem_Battery(const DataGenericItem_Battery &copyObj);

    DataGenericItem_Battery(const mavlink_battery_status_t &copyObj);

    void setBatteryVoltage(const double &voltage){
        this->_voltage = voltage;
    }
    double getBatteryVoltage() const{
        return _voltage;
    }

    void setBatteryCurrent(const double &current){
        this->_current = current;
    }
    double getBatteryCurrent() const{
        return _current;
    }

    void setBatteryRemaining(const double &batteryRemaing){
        this->_batteryRemaing = batteryRemaing;
    }
    double getBatteryRemaining() const{
        return _batteryRemaing;
    }

    mavlink_battery_status_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_Battery &rhs)
    {
        this->_voltage = rhs._voltage;
        this->_current = rhs._current;
        this->_batteryRemaing = rhs._batteryRemaing;
    }

    bool operator == (const DataGenericItem_Battery &rhs) {
        if(fabs(this->_voltage - rhs._voltage) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->_current - rhs._current) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        if(fabs(this->_batteryRemaing - rhs._batteryRemaing) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Battery &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Vehicle Battery( Voltage: "<<_voltage<<", Current: "<<_current<<", Remaining %: "<<_batteryRemaing<<")";
        return out;
    }

protected:
    double _voltage;
    double _current;
    double _batteryRemaing;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FUEL_H
