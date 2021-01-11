#include "topic_trackangle.h"

namespace mace{

namespace measurement_topics{

const char TopicName_Trackangle[] = "TOPIC_TRACKANGLE";

const MaceCore::TopicComponentStructure Structure_Trackangle = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("angle");
    structure.AddTerminal<int>("targetID");
    return structure;
}();

MaceCore::TopicDatagram Topic_TrackAngle::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("angle", m_TrackAngleObj.getAngle());
    datagram.AddTerminal<int>("targetID", m_TrackAngleObj.getTarget());
    return datagram;
}

void Topic_TrackAngle::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_TrackAngleObj.setAngle(datagram.GetTerminal<double>("angle"));
    m_TrackAngleObj.setTarget(datagram.GetTerminal<int>("targetID"));
}

QJsonObject Topic_TrackAngle::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["angle"] = m_TrackAngleObj.getAngle()*180/M_PI;
    json["targetID"] = m_TrackAngleObj.getTarget();
    return json;
}

void Topic_TrackAngle::fromJSON(const QJsonDocument &inputJSON)
{
    m_TrackAngleObj.setTarget(inputJSON.object().value("targetID").toInt());
    m_TrackAngleObj.setAngle(inputJSON.object().value("angle").toDouble()*M_PI/180);
}

std::string Topic_TrackAngle::toCSV(const std::string &delimiter) const
{
    std::string newline = std::to_string(m_TrackAngleObj.getTarget()) + delimiter + std::to_string(m_TrackAngleObj.getAngle()*180/M_PI);
    return newline;
}

Topic_TrackAngle::Topic_TrackAngle()
{

}

Topic_TrackAngle::Topic_TrackAngle(const mace::measurements::TrackAngle &angleObj)
{
    //copy the contents
    this->m_TrackAngleObj = angleObj;
}

Topic_TrackAngle::Topic_TrackAngle(const Topic_TrackAngle &copy)
{
    this->m_TrackAngleObj = copy.m_TrackAngleObj;
}

mace::measurements::TrackAngle Topic_TrackAngle::getTrackAngleObj() const
{
    return this->m_TrackAngleObj;
}

void Topic_TrackAngle::setTrackAngleObj(const mace::measurements::TrackAngle &angleObj)
{
    this->m_TrackAngleObj = angleObj;
}

void Topic_TrackAngle::TrackAngleFromState(const pose::GeodeticPosition_3D &target, const pose::GeodeticPosition_3D &ownPose, const pose::Rotation_3D &ownAttitude)
{
    this->m_TrackAngleObj.calculateFromState(target,ownPose,ownAttitude);
}



} //end of namespace measurement_topics
} //end of namespace pose

