#ifndef TOPIC_TRACKANGLE_H
#define TOPIC_TRACKANGLE_H

#include "common/class_forward.h"

#include "data/i_topic_component_data_object.h"

#include "data/jsonconverter.h"

#include "base/measurements/trackangle.h"

namespace mace {
namespace measurement_topics {

extern const char TopicName_Trackangle[];
extern const MaceCore::TopicComponentStructure Structure_Trackangle;

MACE_CLASS_FORWARD(Topic_TrackAngle);

class Topic_TrackAngle : public Data::NamedTopicComponentDataObject<TopicName_Trackangle, &Structure_Trackangle>, public JSONConverter
{
public:

    Topic_TrackAngle();

    Topic_TrackAngle(const Topic_TrackAngle &copyObj);

    Topic_TrackAngle(const mace::measurements::TrackAngle &angleObj);

    ~Topic_TrackAngle() override = default;

public:
    MaceCore::TopicDatagram GenerateDatagram() const override;

    void CreateFromDatagram(const MaceCore::TopicDatagram &datagram) override;

    mace::measurements::TrackAngle getTrackAngleObj() const;

    void setTrackAngleObj(const mace::measurements::TrackAngle &angleObj);

    void TrackAngleFromState(const pose::GeodeticPosition_3D &target, const pose::GeodeticPosition_3D &ownPose, const pose::Rotation_3D &ownAttitude);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;
private:
    mace::measurements::TrackAngle m_TrackAngleObj;
};

} //end of namespace measurement_topics
} //end of namespace pose


#endif // TOPIC_TRACKANGLE_H
