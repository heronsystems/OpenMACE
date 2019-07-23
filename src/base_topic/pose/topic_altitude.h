#ifndef TOPIC_ALTITUDE_H
#define TOPIC_ALTITUDE_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/base_altitude.h""

namespace pose {
namespace BaseTopic {

extern const char TopicName_Altitude[];

extern const MaceCore::TopicComponentStructure Structure_Altitude;

class Topic_Altitude : public Data::NamedTopicComponentDataObject<TopicName_Altitude, &Structure_Altitude>
{

public:

    Topic_Altitude();

    Topic_Altitude(const Topic_Altitude &copyObj);

    Topic_Altitude(const mace::pose::Altitude &altObj);

    virtual ~Topic_Altitude()
    {

    }

public:
    MaceCore::TopicDatagram GenerateDatagram() const override;

    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

    mace::pose::Altitude getAltitudeObj() const;

private:
    mace::pose::Altitude altitudeObj;
};

} //end of namespace BaseTopic
} //end of namespace pose


#endif // TOPIC_ALTITUDE_H
