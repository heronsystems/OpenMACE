#ifndef VEHICLE_PATH_LINEAR_TOPIC_H
#define VEHICLE_PATH_LINEAR_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "base/vehicle/vehicle_path_linear.h"

namespace mace{
namespace topic{

extern const char Vehicle_Linear_Path_Topic_name[];
extern const MaceCore::TopicComponentStructure Vehicle_Linear_Path_Topic_structure;

MACE_CLASS_FORWARD(Vehicle_Path_Linear_Topic);

class Vehicle_Path_Linear_Topic :public Data::NamedTopicComponentDataObject<Vehicle_Linear_Path_Topic_name, &Vehicle_Linear_Path_Topic_structure>
{
public:
    MaceCore::TopicDatagram GenerateDatagram() const override;
    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

public:

    Vehicle_Path_Linear_Topic() = default;

    Vehicle_Path_Linear_Topic(const VehiclePath_Linear &obj)
    {
        this->setPath(obj);
    }

    void setPath(const VehiclePath_Linear &obj)
    {
        this->m_Path = obj;
    }

    VehiclePath_Linear getPath() const
    {
        return m_Path;
    }

private:
    VehiclePath_Linear m_Path;
};

} //end of namepsace topic
} //end of namespace mace

#endif // VEHICLE_PATH_LINEAR_TOPIC_H
