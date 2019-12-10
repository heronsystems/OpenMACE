#include "mission_item_request_topic.h"

namespace MissionTopic{

const char MissionItemRequestTopic_name[] = "MissionItemRequest";
const MaceCore::TopicComponentStructure MissionItemRequestTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<unsigned int>("vehicleID");
    structure.AddTerminal<unsigned int>("missionItemIndex");
    return structure;
}();

MaceCore::TopicDatagram MissionItemRequestTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<unsigned int>("vehicleID",vehicleID);
    datagram.AddTerminal<unsigned int>("missionItemIndex",missionItemIndex);
    return datagram;
}

void MissionItemRequestTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    vehicleID = datagram.GetTerminal<unsigned int>("vehicleID");
    missionItemIndex = datagram.GetTerminal<unsigned int>("missionItemIndex");
}


MissionItemRequestTopic::MissionItemRequestTopic(const unsigned int &vehicleID, const unsigned int &missionItemIndex)
{
    this->vehicleID = vehicleID;
    this->missionItemIndex = missionItemIndex;
}

} //end of namespace MissionTopic
