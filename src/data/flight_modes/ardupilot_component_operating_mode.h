#ifndef ARDUPILOT_COMPONENT_OPERATING_MODE_H
#define ARDUPILOT_COMPONENT_OPERATING_MODE_H

#include <iostream>
#include <map>
#include <list>
#include <string>
#include "common/common.h"

#include "data/vehicle_types.h"
#include "mavlink.h"

class ARDUPILOTComponent_OperatingMode
{

public:
    ARDUPILOTComponent_OperatingMode() = default;

    virtual ~ARDUPILOTComponent_OperatingMode() = default;

    virtual std::string parseMAVLINK(const mavlink_heartbeat_t &msg) = 0;

    virtual uint32_t getFlightModeFromString(const std::string &modeString) const = 0;

    static std::map<uint32_t, std::string> getAvailableFlightModes() {
        std::map<uint32_t, std::string> map;
        return map;
    }

    static std::string getFlightModeStr(const uint8_t &mode) {
        UNUSED(mode);
        return "UNKNOWN";
    }

protected:
    uint32_t _currentMode = 0;
};

#endif // ARDUPILOT_COMPONENT_OPERATING_MODE_H
