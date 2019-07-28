#ifndef TOPIC_AGENTORIENTATION_H
#define TOPIC_AGENTORIENTATION_H


#include "data/i_topic_component_data_object.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"

namespace pose {
namespace BaseTopic {

extern const char TopicName_AgentOrientation[];

extern const MaceCore::TopicComponentStructure Structure_AgentOrientation;

class Topic_AgentOrientation : public Data::NamedTopicComponentDataObject<TopicName_AgentOrientation, &Structure_AgentOrientation>
{

public:
    Topic_AgentOrientation();

    Topic_AgentOrientation(const Topic_AgentOrientation &copyObj);

    Topic_AgentOrientation(const mace::pose::Abstract_GeodeticPosition *posObj);

    virtual ~Topic_AgentOrientation()
    {
        delete m_PositionObject;
    }

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    mace::pose::Abstract_GeodeticPosition* getPositionObj() const;

private:
    mace::pose::Abstract_GeodeticPosition* m_PositionObject;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_AGENTORIENTATION_H
