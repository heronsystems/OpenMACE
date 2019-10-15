#include "mission_item_reached_topic.h"

namespace MissionTopic{

const char MissionItemReachedTopic_name[] = "MissionItemReached";
const MaceCore::TopicComponentStructure MissionItemReachedTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MissionItem::MissionKey>("key");
    structure.AddTerminal<unsigned int>("index");
    return structure;
}();

MaceCore::TopicDatagram MissionItemReachedTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MissionItem::MissionKey>("key",key);
    datagram.AddTerminal<unsigned int>("index",indexAchieved);
    return datagram;
}

void MissionItemReachedTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    key = datagram.GetTerminal<MissionItem::MissionKey>("key");
    indexAchieved = datagram.GetTerminal<unsigned int>("index");
}

MissionItemReachedTopic::MissionItemReachedTopic()
{

}

MissionItemReachedTopic::MissionItemReachedTopic(const MissionItem::MissionItemAchieved &achievedItem)
{
    this->key = achievedItem.getMissionKey();
    this->indexAchieved = achievedItem.getMissionAchievedIndex();
}

MissionItemReachedTopic::MissionItemReachedTopic(const MissionItemReachedTopic &copyObj)
{
    this->key = copyObj.getMissionKey();
    this->indexAchieved = copyObj.getMissionAchievedIndex();
}
} //end of namespace MissionTopic
