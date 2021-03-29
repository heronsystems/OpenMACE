#include "command_mission_waypoint.h"

namespace DataVehicleCommands {
    template<class T>
    CommandTypes CommandMissionWaypoint<T>::getCommandType() const
    {
        return CommandTypes::MISSION;
    }

    template<class T>
    MissionItemTypes CommandMissionWaypoint<T>::getMissionType() const
    {
        return MissionItemTypes::WAYPOINT;
    }

    template<class T>
    std::string CommandMissionWaypoint<T>::getDescription() const
    {
        return "This is a waypoint mission item";
    }

    template<>
    CommandMissionWaypoint<DataState::StateGlobalPosition>::CommandMissionWaypoint(){
        m_PositionFrame = Data::PositionalFrame::GLOBAL;
    }

    template<>
    CommandMissionWaypoint<DataState::StateLocalPosition>::CommandMissionWaypoint(){
        m_PositionFrame = Data::PositionalFrame::LOCAL;
    }

    template <>
    void CommandMissionWaypoint<DataState::StateGlobalPosition>::setLocation(const DataState::StateGlobalPosition &location)
    {
        m_Location.latitude = location.latitude;
        m_Location.longitude = location.longitude;
        m_Location.altitude = location.altitude;
    }

    template <>
    void CommandMissionWaypoint<DataState::StateGlobalPosition>::setLocation(const double &latitude, const double &longitude, const double &altitude)
    {
        m_Location.latitude = latitude;
        m_Location.longitude = longitude;
        //fix the altitude component
    }

    template <>
    void CommandMissionWaypoint<DataState::StateLocalPosition>::setLocation(const DataState::StateLocalPosition &location)
    {
        m_Location.x = location.x;
        m_Location.y = location.y;
        m_Location.z = location.z;
    }
}
