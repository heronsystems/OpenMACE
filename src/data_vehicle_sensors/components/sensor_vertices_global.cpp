#include "sensor_vertices_global.h"

namespace DataVehicleSensors
{
const char SensorVerticesGlobal_Name[] = "SensorVerticesGlobal";

const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<std::vector<mace::pose::GeodeticPosition_2D>>("SensorVertices");
    return structure;
}();


MaceCore::TopicDatagram SensorVertices_Global::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<std::vector<mace::pose::GeodeticPosition_2D>>("SensorVertices",verticeLocations);
    return datagram;
}

void SensorVertices_Global::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    verticeLocations = datagram.GetTerminal<std::vector<mace::pose::GeodeticPosition_2D>>("SensorVertices");
}

SensorVertices_Global::SensorVertices_Global()
{
    this->sensorName = "";
}

SensorVertices_Global::SensorVertices_Global(const std::string &sensorName)
{
    this->sensorName = sensorName;
}

std::vector<mace::pose::GeodeticPosition_2D> SensorVertices_Global::getSensorVertices() const
{
    return(verticeLocations);
}

void SensorVertices_Global::setSensorVertices(const std::vector<mace::pose::GeodeticPosition_2D> &verticeVector)
{
    verticeLocations = verticeVector;
}

} //end of namespace DataVehicleSensors
