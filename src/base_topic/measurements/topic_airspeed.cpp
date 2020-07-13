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
    m_SpeedObj.updateSpeedType(mace::measurements::Speed::SpeedTypes::AIRSPEED);
    m_SpeedObj.setSpeed(datagram.GetTerminal<double>("Speed"));
}

QJsonObject Topic_AirSpeed::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["airspeed"] = getSpeedObj().getSpeed();
    return json;
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

