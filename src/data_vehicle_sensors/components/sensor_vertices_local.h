#ifndef SENSOR_VERTICES_LOCAL_H
#define SENSOR_VERTICES_LOCAL_H

#include <list>
#include <string>
#include <cmath>

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_item/state_local_position.h"

namespace DataVehicleSensors
{

extern const char SensorVerticesLocal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesLocal_Structure;

class SensorVertices_Local : public Data::NamedTopicComponentDataObject<SensorVerticesLocal_Name, &SensorVerticesLocal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Local();
    SensorVertices_Local(const std::string &sensorName);

public:
    std::vector<DataState::StateLocalPosition> getSensorVertices() const;
    void setSensorVertices(const std::vector<DataState::StateLocalPosition> &verticeVector);

private:
    std::string sensorName;
    Data::CoordinateFrameType coordinateFrame;
    std::vector<DataState::StateLocalPosition> verticeLocations;
};

} //end of namespace DataVehicleSensors
#endif // SENSOR_VERTICES_LOCAL_H
