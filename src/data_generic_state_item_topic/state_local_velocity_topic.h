#ifndef STATE_LOCAL_VELOCITY_TOPIC_H
#define STATE_LOCAL_VELOCITY_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_local_velocity.h"

namespace DataStateTopic {

extern const char LocalVelocityTopic_name[];
extern const MaceCore::TopicComponentStructure LocalVelocityTopic_structure;

class StateLocalVelocityTopic : public DataState::StateLocalVelocity, public Data::NamedTopicComponentDataObject<LocalVelocityTopic_name, &LocalVelocityTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateLocalVelocityTopic();
    StateLocalVelocityTopic(const DataState::StateLocalVelocity &copyObj);
};

} //end of namespace DataStateTopic
#endif // STATE_LOCAL_VELOCITY_TOPIC_H
