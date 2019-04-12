#ifndef TOPIC_COMPONENTS_ALTITUDE_H
#define TOPIC_COMPONENTS_ALTITUDE_H

#include "../i_topic_component_data_object.h"

#include "../reference_altitude.h"

#include "../topic_prototypes/altitude.h"

namespace Data {

namespace TopicComponents
{

extern const char TopicComponts_Altitude_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_Altitude_structure;

class Altitude : public TopicComponentPrototypes::Altitude, public Data::NamedTopicComponentDataObject<TopicComponts_Altitude_name, &TopicComponts_Altitude_structure>
{
};


} // TopicComponents

} // Data


#endif // TOPIC_COMPONENTS_ALTITUDE_H
