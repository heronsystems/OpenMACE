#ifndef TOPIC_SPEED_H
#define TOPIC_SPEED_H

#include "common/class_forward.h"

#include "data/i_topic_component_data_object.h"

#include "data/jsonconverter.h"

#include "base/measurements/base_speed.h"

namespace mace {
namespace measurement_topics {

extern const char TopicName_Airspeed[];
extern const MaceCore::TopicComponentStructure Structure_Airspeed;

MACE_CLASS_FORWARD(Topic_AirSpeed);

class Topic_AirSpeed : public Data::NamedTopicComponentDataObject<TopicName_Airspeed, &Structure_Airspeed>, public JSONConverter
{
public:

    Topic_AirSpeed();

    Topic_AirSpeed(const Topic_AirSpeed &copyObj);

    Topic_AirSpeed(const mace::measurements::Speed &speedObj);

    ~Topic_AirSpeed() override = default;

public:
    MaceCore::TopicDatagram GenerateDatagram() const override;

    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

    mace::measurements::Speed getSpeedObj() const;

    void setSpeedObj(const mace::measurements::Speed &speedObj);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;
private:
    mace::measurements::Speed m_SpeedObj;
};

extern const char TopicName_Groundspeed[];
extern const MaceCore::TopicComponentStructure Structure_Groundspeed;

MACE_CLASS_FORWARD(Topic_GroundSpeed);

class Topic_GroundSpeed : public Data::NamedTopicComponentDataObject<TopicName_Groundspeed, &Structure_Groundspeed>
{
public:

    Topic_GroundSpeed();

    Topic_GroundSpeed(const Topic_GroundSpeed &copyObj);

    Topic_GroundSpeed(const mace::measurements::Speed &speedObj);

    ~Topic_GroundSpeed() override = default;

public:
    MaceCore::TopicDatagram GenerateDatagram() const override;

    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

    mace::measurements::Speed getSpeedObj() const;

    void setSpeedObj(const mace::measurements::Speed &speedObj);

private:
    mace::measurements::Speed m_SpeedObj;
};

} //end of namespace measurement_topics
} //end of namespace pose


#endif // TOPIC_SPEED_H
