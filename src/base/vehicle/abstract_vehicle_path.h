#ifndef ABSTRACT_VEHICLE_PATH_H
#define ABSTRACT_VEHICLE_PATH_H

#include <string>
#include <thread>
#include <iostream>

class AbstractVehiclePath 
{
public:

    AbstractVehiclePath()
    {

    }

    AbstractVehiclePath(const AbstractVehiclePath &copy)
    {
        _agentID = copy._agentID;
    }

    ~AbstractVehiclePath() = default;

    void setAgentID(const std::string &ID)
    {
        _agentID = ID;
    }

    std::string getAgentID() const
    {
        return _agentID;
    }

public:

protected:
    std::string _agentID = "";

};

#endif // ABSTRACT_VEHICLE_PATH_H
