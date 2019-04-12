#ifndef TOPIC_COMPONENTS_POSITION_GLOBAL_H
#define TOPIC_COMPONENTS_POSITION_GLOBAL_H

#include "../i_topic_component_data_object.h"

#include "../reference_altitude.h"

#include "../topic_prototypes/altitude.h"
#include "../topic_prototypes/position_georeference.h"

namespace Data {

namespace TopicComponents
{

extern const char TopicComponts_PositionGlobal_name[];
extern const MaceCore::TopicComponentStructure TopicComponts_PositionGlobal_structure;

class PositionGlobal : public TopicComponentPrototypes::Altitude, public TopicComponentPrototypes::PositionGeoreference, public Data::NamedTopicComponentDataObject<TopicComponts_PositionGlobal_name, &TopicComponts_PositionGlobal_structure>
{
private:



public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    PositionGlobal(const double &altitude, const ReferenceAltitude &alt_ref, const double &lattitude, const double &longitude, const ReferenceGeoCoords &geo_ref);

    PositionGlobal(const PositionGlobal &copyObj);
};


} // TopicComponents

} // Data


#endif // TOPIC_COMPONENTS_POSITION_GLOBAL_H
