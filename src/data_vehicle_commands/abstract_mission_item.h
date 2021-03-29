#ifndef ABSTRACT_MISSION_ITEM_H
#define ABSTRACT_MISSION_ITEM_H

#include "command_types.h"

namespace DataVehicleCommands {

enum class MissionItemTypes{
    WAYPOINT
};

class AbstractMissionItem
{
public:
    virtual CommandTypes getCommandType() const = 0;

    virtual MissionItemTypes getMissionType() const = 0;

    virtual std::string getDescription() const = 0;
};

} //end of namespace DataVehicleCommands

#endif // ABSTRACT_MISSION_ITEM_H
