#ifndef TOPIC_ROTATIONALVELOCITY_H
#define TOPIC_ROTATIONALVELOCITY_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/velocity_interface_rotational.h"

namespace mace {
namespace pose_topics {

extern const char TopicName_RotationalVelocity[];

extern const MaceCore::TopicComponentStructure Structure_RotationalVelocity;

class Topic_RotationalVelocity : public Data::NamedTopicComponentDataObject<TopicName_RotationalVelocity, &Structure_RotationalVelocity>
{

public:

    Topic_RotationalVelocity();

    Topic_RotationalVelocity(const Topic_RotationalVelocity &copyObj);

    Topic_RotationalVelocity(const mace::pose::Velocity_Rotation3D &velObj);

    virtual ~Topic_RotationalVelocity()
    {

    }

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    mace::pose::Velocity_Rotation3D getVelocityObj() const;

private:
    mace::pose::Velocity_Rotation3D m_VelocityObj;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_ROTATIONALVELOCITY_H
