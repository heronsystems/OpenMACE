#ifndef STATE_AIRSPEED_TOPIC_H
#define STATE_AIRSPEED_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_state_item/state_airspeed.h"

namespace DataStateTopic {

extern const char AirspeedTopic_name[];
extern const MaceCore::TopicComponentStructure AirspeedTopic_structure;

class StateAirspeedTopic : public DataState::StateAirspeed, public Data::NamedTopicComponentDataObject<AirspeedTopic_name, &AirspeedTopic_structure>
{
public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateAirspeedTopic();
    StateAirspeedTopic(const DataState::StateAirspeed &copyObj);
};

} //end of namespace DataStateTopic
#endif // STATE_AIRSPEED_TOPIC_H
