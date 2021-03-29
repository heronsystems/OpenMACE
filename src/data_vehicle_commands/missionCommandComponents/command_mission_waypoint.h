#ifndef COMMAND_MISSION_WAYPOINT_H
#define COMMAND_MISSION_WAYPOINT_H

#include "data_vehicle_commands/abstract_mission_item.h"

#include "data/coordinate_frame.h"
#include "data/positional_coordinate_frame.h"
#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace DataVehicleCommands {

template<class T>
class CommandMissionWaypoint : public AbstractMissionItem
{
public:
    virtual CommandTypes getCommandType() const;

    virtual MissionItemTypes getMissionType() const;

    virtual std::string getDescription() const;

public:
    CommandMissionWaypoint();

    Data::PositionalFrame getLocationType(){
        return m_PositionFrame;
    }

    void setLocation(const T &location);
    void setLocation(const double &latitude, const double &longitude, const double &altitude);
    T getLocation();

private:
    Data::PositionalFrame m_PositionFrame;
    T m_Location;

};

}

#endif // COMMAND_MISSION_WAYPOINT_H
