#include "sensor_vertices_global.h"

namespace DataVehicleSensors
{
const char SensorVerticesGlobal_Name[] = "SensorVerticesGlobal";

const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("SensorName");
    structure.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    structure.AddTerminal<std::vector<DataState::StateGlobalPosition>>("SensorVertices");
    return structure;
}();


MaceCore::TopicDatagram SensorVertices_Global::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("SensorName",sensorName);
    datagram.AddTerminal<Data::CoordinateFrameType>("CoordinateFrame",coordinateFrame);
    datagram.AddTerminal<std::vector<DataState::StateGlobalPosition>>("SensorVertices",verticeLocations);
    return datagram;
}

void SensorVertices_Global::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    sensorName = datagram.GetTerminal<std::string>("SensorName");
    coordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    verticeLocations = datagram.GetTerminal<std::vector<DataState::StateGlobalPosition>>("SensorVertices");
}

SensorVertices_Global::SensorVertices_Global()
{
    this->coordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
    this->sensorName = "";
}

SensorVertices_Global::SensorVertices_Global(const std::string &sensorName)
{
    this->coordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
    this->sensorName = sensorName;
}

std::vector<DataState::StateGlobalPosition> SensorVertices_Global::getSensorVertices() const
{
    return(verticeLocations);
}

void SensorVertices_Global::setSensorVertices(const std::vector<DataState::StateGlobalPosition> &verticeVector)
{
    verticeLocations = verticeVector;
}

} //end of namespace DataVehicleSensors
