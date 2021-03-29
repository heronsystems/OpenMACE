#ifndef STATE_ATTITUDE_TOPIC_H
#define STATE_ATTITUDE_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_item/state_attitude.h"

namespace DataStateTopic {

extern const char AttitudeTopic_name[];
extern const MaceCore::TopicComponentStructure AttitudeTopic_structure;

class StateAttitudeTopic : public DataState::StateAttitude, public Data::NamedTopicComponentDataObject<AttitudeTopic_name, &AttitudeTopic_structure>
{
public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateAttitudeTopic();
    StateAttitudeTopic(const DataState::StateAttitude &copyObj);
};

} //end of namespace DataStateTopic

#endif // STATE_ATTITUDE_TOPIC_H
