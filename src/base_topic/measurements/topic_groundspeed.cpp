#include "topic_speed.h"

namespace mace{

namespace measurement_topics{

const char TopicName_Groundspeed[] = "TOPIC_GROUNDSPEED";

const MaceCore::TopicComponentStructure Structure_Groundspeed = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("Speed");
    return structure;
}();

MaceCore::TopicDatagram Topic_GroundSpeed::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("Speed", m_SpeedObj.getSpeed());
    return datagram;
}

void Topic_GroundSpeed::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    m_SpeedObj.updateSpeedType(mace::measurements::Speed::SpeedTypes::GROUNDSPEED);
    m_SpeedObj.setSpeed(datagram.GetTerminal<double>("Speed"));
}

Topic_GroundSpeed::Topic_GroundSpeed()
{

}

Topic_GroundSpeed::Topic_GroundSpeed(const mace::measurements::Speed &speedObj)
{
    //copy the contents
    this->m_SpeedObj = speedObj;
}

Topic_GroundSpeed::Topic_GroundSpeed(const Topic_GroundSpeed &copy)
{
    this->m_SpeedObj = copy.m_SpeedObj;
}

mace::measurements::Speed Topic_GroundSpeed::getSpeedObj() const
{
    return this->m_SpeedObj;
}

void Topic_GroundSpeed::setSpeedObj(const mace::measurements::Speed &speedObj)
{
    this->m_SpeedObj = speedObj;
}

} //end of namespace measurement_topics
} //end of namespace pose

