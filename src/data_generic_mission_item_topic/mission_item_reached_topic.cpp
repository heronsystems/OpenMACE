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

MissionItemReachedTopic::MissionItemReachedTopic(const MissionItemReachedTopic &copyObj) :
    MissionItem::MissionItemAchieved()
{
    this->key = copyObj.getMissionKey();
    this->indexAchieved = copyObj.getMissionAchievedIndex();
}

QJsonObject MissionItemReachedTopic::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["itemIndex"] = static_cast<int>(getMissionAchievedIndex());    
    return json;
}

} //end of namespace MissionTopic
