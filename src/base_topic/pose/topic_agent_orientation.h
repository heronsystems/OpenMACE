#ifndef TOPIC_AGENTORIENTATION_H
#define TOPIC_AGENTORIENTATION_H

#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

#include "base/pose/rotation_2D.h"
#include "base/pose/rotation_3D.h"

namespace mace {
namespace pose_topics {

extern const char TopicName_AgentOrientation[];

extern const MaceCore::TopicComponentStructure Structure_AgentOrientation;

MACE_CLASS_FORWARD(Topic_AgentOrientation);

class Topic_AgentOrientation : public Data::NamedTopicComponentDataObject<TopicName_AgentOrientation, &Structure_AgentOrientation>, public JSONConverter
{

public:
    Topic_AgentOrientation();

    Topic_AgentOrientation(const Topic_AgentOrientation &copyObj);

    Topic_AgentOrientation(const mace::pose::AbstractRotation *obj);

    virtual ~Topic_AgentOrientation()
    {
        delete m_RotationObj; m_RotationObj = nullptr;
    }

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    mace::pose::AbstractRotation* getRotationObj() const;

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

private:
    mace::pose::AbstractRotation* m_RotationObj;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_AGENTORIENTATION_H
