#ifndef CARTESIAN_2D_TOPIC_H
#define CARTESIAN_2D_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/cartesian_position_2D.h"

namespace mace{
namespace poseTopic{

extern const char Cartesian_2D_Topic_name[];
extern const MaceCore::TopicComponentStructure Cartesian_2D_Topic_structure;

class Cartesian_2D_Topic :public Data::NamedTopicComponentDataObject<Cartesian_2D_Topic_name, &Cartesian_2D_Topic_structure>
{
public:
    MaceCore::TopicDatagram GenerateDatagram() const override;
    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

public:

    Cartesian_2D_Topic() = default;

    Cartesian_2D_Topic(const pose::CartesianPosition_2D &obj)
    {
        this->setPose(obj);
    }

    void setPose(const pose::CartesianPosition_2D &obj)
    {
        this->pose = obj;
    }

    pose::CartesianPosition_2D getPose() const
    {
        return pose;
    }

private:
    pose::CartesianPosition_2D pose;
};

} //end of namepsace poseTopic
} //end of namespace mace

#endif // CARTESIAN_2D_TOPIC_H
