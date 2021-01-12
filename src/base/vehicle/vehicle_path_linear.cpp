#include "vehicle_path_linear.h"

VehiclePath_Linear::VehiclePath_Linear():
    AbstractVehiclePath()
{
    setAgentID("agent_0");
    m_vertices.clear();
}

VehiclePath_Linear::VehiclePath_Linear(const VehiclePath_Linear &copy) : 
    AbstractVehiclePath(copy)
{
    this->m_vertices = copy.m_vertices;
}

// Getters and setters:
void VehiclePath_Linear::setVertices(const std::vector<pose::GeodeticPosition_3D> &vertices)
{
    m_vertices = vertices;
}

std::vector<pose::GeodeticPosition_3D> VehiclePath_Linear::getVertices() const
{
    return m_vertices;
}
