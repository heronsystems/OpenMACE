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

    virtual std::map<uint32_t, std::string> getAvailableFlightModes() const = 0;

protected:
    uint32_t _currentMode = 0;
};

#endif // ARDUPILOT_COMPONENT_OPERATING_MODE_H
