#ifndef TOPIC_CARTESIAN_VELOCITY_H
#define TOPIC_CARTESIAN_VELOCITY_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/base_velocity.h"

namespace pose {
namespace BaseTopic {

extern const char TopicName_CartesianVelocity[];

extern const MaceCore::TopicComponentStructure Structure_CartesianVelocity;

class Topic_CartesianVelocity : public Data::NamedTopicComponentDataObject<TopicName_CartesianVelocity, &Structure_CartesianVelocity>
{

public:

    Topic_CartesianVelocity();

    Topic_CartesianVelocity(const Topic_CartesianVelocity &copyObj);

    Topic_CartesianVelocity(const mace::pose::Abstract_Velocity* velObj);

    virtual ~Topic_CartesianVelocity()
    {
        delete m_VelocityObj;
    }

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    mace::pose::Abstract_Velocity* getVelocityObj() const;

private:
    mace::pose::Abstract_Velocity* m_VelocityObj;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_CARTESIAN_VELOCITY_H
