#include "topic_speed.h"

namespace mace{

namespace measurement_topics{

const char TopicName_Airspeed[] = "TOPIC_AIRSPEED";

const MaceCore::TopicComponentStructure Structure_Airspeed = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("Speed");
    return structure;
}();

MaceCore::TopicDatagram Topic_AirSpeed::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("Speed", m_SpeedObj.getSpeed());
    return datagram;
}

void Topic_AirSpeed::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_SpeedObj.updateSpeedType(SPEED_TYPE::SPEED_TYPE_AIRSPEED);
    m_SpeedObj.setSpeed(datagram.GetTerminal<double>("Speed"));
}

QJsonObject Topic_AirSpeed::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["airspeed"] = getSpeedObj().getSpeed();
    return json;
}

void Topic_AirSpeed::fromJSON(const QJsonDocument &inputJSON)
{
    m_SpeedObj.setSpeed(inputJSON.object().value("airspeed").toDouble());
}

std::string Topic_AirSpeed::toCSV(const std::string &delimiter) const
{
    UNUSED(delimiter);
    std::string newline = std::to_string(m_SpeedObj.getSpeed());
    return newline;
}

Topic_AirSpeed::Topic_AirSpeed()
{

}

Topic_AirSpeed::Topic_AirSpeed(const mace::measurements::Speed &speedObj)
{
    //copy the contents
    this->m_SpeedObj = speedObj;
}

Topic_AirSpeed::Topic_AirSpeed(const Topic_AirSpeed &copy)
{
    this->m_SpeedObj = copy.m_SpeedObj;
}

mace::measurements::Speed Topic_AirSpeed::getSpeedObj() const
{
    return this->m_SpeedObj;
}

void Topic_AirSpeed::setSpeedObj(const mace::measurements::Speed &speedObj)
{
    this->m_SpeedObj = speedObj;
}



} //end of namespace measurement_topics
} //end of namespace pose

