#ifndef TOPIC_CARTESIAN_VELOCITY_H
#define TOPIC_CARTESIAN_VELOCITY_H

#include "data/i_topic_component_data_object.h"

#include "base/pose/velocity_helper.h"
#include "data/jsonconverter.h"

namespace mace {
namespace pose_topics {

extern const char TopicName_CartesianVelocity[];

extern const MaceCore::TopicComponentStructure Structure_CartesianVelocity;

class Topic_CartesianVelocity : public Data::NamedTopicComponentDataObject<TopicName_CartesianVelocity, &Structure_CartesianVelocity>, public JSONConverter
{

public:

    Topic_CartesianVelocity();

    Topic_CartesianVelocity(const Topic_CartesianVelocity &copyObj);

    Topic_CartesianVelocity(const mace::pose::Velocity* velObj);

    virtual ~Topic_CartesianVelocity()
    {
        delete m_VelocityObj;
    }

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    mace::pose::Velocity* getVelocityObj() const;

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;
private:
    mace::pose::Velocity* m_VelocityObj;
};

} //end of namespace BaseTopic
} //end of namespace pose

#endif // TOPIC_CARTESIAN_VELOCITY_H
