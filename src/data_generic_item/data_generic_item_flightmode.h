#ifndef DATA_GENERIC_ITEM_FLIGHTMODE_H
#define DATA_GENERIC_ITEM_FLIGHTMODE_H

#include <string>
#include <stdint.h>

#include "mace.h"

#include "data/vehicle_types.h"
#include "data/autopilot_types.h"


namespace DataGenericItem {

class DataGenericItem_FlightMode
{

public:
    DataGenericItem_FlightMode();

    DataGenericItem_FlightMode(const std::string &mode);

    DataGenericItem_FlightMode(const DataGenericItem_FlightMode &copyObj);

    DataGenericItem_FlightMode(const mace_vehicle_mode_t &copyObj);

public:

    void setFlightMode(const std::string &flightMode) {
        this->flightModeString = flightMode;
    }

    std::string getFlightModeString() const {
        return (flightModeString);
    }

    mace_vehicle_mode_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_FlightMode &rhs)
    {
        this->flightModeString = rhs.flightModeString;
    }

    bool operator == (const DataGenericItem_FlightMode &rhs) {
        if(this->flightModeString != rhs.flightModeString) {
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_FlightMode &rhs) {
        return !(*this == rhs);
    }

protected:
    std::string flightModeString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_FLIGHTMODE_H
