#ifndef STATE_GLOBAL_VELOCITY_TOPIC_H
#define STATE_GLOBAL_VELOCITY_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_global_velocity.h"

namespace DataStateTopic {

extern const char GlobalVelocityTopic_name[];
extern const MaceCore::TopicComponentStructure GlobalVelocityTopic_structure;

class StateGlobalVelocityTopic : public DataState::StateGlobalVelocity, public Data::NamedTopicComponentDataObject<GlobalVelocityTopic_name, &GlobalVelocityTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateGlobalVelocityTopic();
    StateGlobalVelocityTopic(const DataState::StateGlobalVelocity &copyObj);
};

} //end of namespace DataStateTopic

#endif // STATE_GLOBAL_VELOCITY_TOPIC_H
