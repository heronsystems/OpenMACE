#include "state_airspeed_topic.h"

namespace DataStateTopic {

const char AirspeedTopic_name[] = "Vehicle Airspeed";
const MaceCore::TopicComponentStructure AirspeedTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("airspeed");
    return structure;
}();

MaceCore::TopicDatagram StateAirspeedTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("airspeed", airspeed);
    return datagram;
}

void StateAirspeedTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    airspeed = datagram.GetTerminal<double>("airspeed");
}

StateAirspeedTopic::StateAirspeedTopic():
    DataState::StateAirspeed()
{

}

StateAirspeedTopic::StateAirspeedTopic(const DataState::StateAirspeed &copyObj):
    DataState::StateAirspeed(copyObj)
{

}

} //end of namespace DataStateTopic
