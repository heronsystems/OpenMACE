#include "sensor_vertices_local.h"

namespace DataVehicleSensors
{
const char SensorVerticesLocal_Name[] = "SensorVerticesLocal";

const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<std::vector<mace::pose::CartesianPosition_2D>>("SensorVertices");
    return structure;
}();

MaceCore::TopicDatagram SensorVertices_Local::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<std::vector<mace::pose::CartesianPosition_2D>>("SensorVertices",verticeLocations);
    return datagram;
}

void SensorVertices_Local::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    verticeLocations = datagram.GetTerminal<std::vector<mace::pose::CartesianPosition_2D>>("SensorVertices");
}


SensorVertices_Local::SensorVertices_Local()
{
    this->sensorName = "";
}

SensorVertices_Local::SensorVertices_Local(const std::string &sensorName)
{
    this->sensorName = sensorName;
}


std::vector<mace::pose::CartesianPosition_2D> SensorVertices_Local::getSensorVertices() const
{
    return(verticeLocations);
}

void SensorVertices_Local::setSensorVertices(const std::vector<mace::pose::CartesianPosition_2D> &verticeVector)
{
    verticeLocations = verticeVector;
}

} //end of namespace DataVehicleSensors
