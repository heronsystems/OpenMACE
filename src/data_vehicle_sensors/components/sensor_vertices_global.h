#ifndef SENSOR_VERTICES_GLOBAL_H
#define SENSOR_VERTICES_GLOBAL_H

#include <list>
#include <string>
#include <cmath>

#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

#include "base/pose/geodetic_position_2D.h"

namespace DataVehicleSensors
{
extern const char SensorVerticesGlobal_Name[];
extern const MaceCore::TopicComponentStructure SensorVerticesGlobal_Structure;

class SensorVertices_Global : public JSONConverter, public Data::NamedTopicComponentDataObject<SensorVerticesGlobal_Name, &SensorVerticesGlobal_Structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
public:
    SensorVertices_Global();
    SensorVertices_Global(const std::string &sensorName);

public:
    std::vector<mace::pose::GeodeticPosition_2D> getSensorVertices() const;
    void setSensorVertices(const std::vector<mace::pose::GeodeticPosition_2D> &verticeVector);
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;
    
private:
    std::string sensorName;
    std::vector<mace::pose::GeodeticPosition_2D> verticeLocations;
};

} //end of namespace DataVehicleSensors

#endif // SENSOR_VERTICES_GLOBAL_H




