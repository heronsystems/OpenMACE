#ifndef SENSOR_VERTICES_GLOBAL_H
#define SENSOR_VERTICES_GLOBAL_H

#include <list>
#include <string>
#include <cmath>

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_item/state_global_position.h"

namespace DataVehicleSensors
{
extern const char SensorVerticesGlobal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure;

class SensorVertices_Global : public Data::NamedTopicComponentDataObject<SensorVerticesGlobal_Name, &SensorVerticesGlobal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Global();
    SensorVertices_Global(const std::string &sensorName);

public:
    std::vector<DataState::StateGlobalPosition> getSensorVertices() const;
    void setSensorVertices(const std::vector<DataState::StateGlobalPosition> &verticeVector);

private:
    std::string sensorName;
    Data::CoordinateFrameType coordinateFrame;
    std::vector<DataState::StateGlobalPosition> verticeLocations;
};

} //end of namespace DataVehicleSensors

#endif // SENSOR_VERTICES_GLOBAL_H




