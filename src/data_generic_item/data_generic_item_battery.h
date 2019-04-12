#ifndef DATA_GENERIC_ITEM_FUEL_H
#define DATA_GENERIC_ITEM_FUEL_H

#include <iostream>

#include "mace.h"

namespace DataGenericItem {

class DataGenericItem_Battery
{
public:
    DataGenericItem_Battery();

    DataGenericItem_Battery(const DataGenericItem_Battery &copyObj);

    DataGenericItem_Battery(const mace_battery_status_t &copyObj);

    void setBatteryVoltage(const double &voltage){
        this->voltage = voltage;
    }
    double getBatteryVoltage() const{
        return voltage;
    }

    void setBatteryCurrent(const double &current){
        this->current = current;
    }
    double getBatteryCurrent() const{
        return current;
    }

    void setBatteryRemaining(const double &batteryRemaing){
        this->batteryRemaing = batteryRemaing;
    }
    double getBatteryRemaining() const{
        return batteryRemaing;
    }

    mace_battery_status_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_Battery &rhs)
    {
        this->voltage = rhs.voltage;
        this->current = rhs.current;
        this->batteryRemaing = rhs.batteryRemaing;
    }

    bool operator == (const DataGenericItem_Battery &rhs) {
        if(this->voltage != rhs.voltage){
            return false;
        }
        if(this->current != rhs.current){
            return false;
        }
        if(this->batteryRemaing != rhs.batteryRemaing){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Battery &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Vehicle Battery( Voltage: "<<voltage<<", Current: "<<current<<", Remaining %: "<<batteryRemaing<<")";
        return out;
    }

protected:
    double voltage;
    double current;
    double batteryRemaing;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FUEL_H
