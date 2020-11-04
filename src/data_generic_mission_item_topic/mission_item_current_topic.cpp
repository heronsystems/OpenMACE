#include "mission_item_current_topic.h"

namespace MissionTopic{

const char MissionItemCurrentTopic_name[] = "MissionItemCurrent";
const MaceCore::TopicComponentStructure MissionItemCurrentTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MissionItem::MissionKey>("key");
    structure.AddTerminal<int>("currentIndex");
    return structure;
}();

MaceCore::TopicDatagram MissionItemCurrentTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MissionItem::MissionKey>("key",key);
    datagram.AddTerminal<int>("currentIndex",indexCurrent);
    return datagram;
}

void MissionItemCurrentTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    key = datagram.GetTerminal<MissionItem::MissionKey>("key");
    indexCurrent = datagram.GetTerminal<int>("currentIndex");
}

QJsonObject MissionItemCurrentTopic::toJSON(const int &vehicleID, const std::string &dataType) const {
    QJsonObject json = toJSON_base(vehicleID,dataType);
    json["missionItemIndex"] = static_cast<int>(getMissionCurrentIndex());
    return json;
}

MissionItemCurrentTopic::MissionItemCurrentTopic()
{

}

MissionItemCurrentTopic::MissionItemCurrentTopic(const MissionItem::MissionItemCurrent &achievedItem)
{
    this->key = achievedItem.getMissionKey();
    this->indexCurrent = achievedItem.getMissionCurrentIndex();
}

MissionItemCurrentTopic::MissionItemCurrentTopic(const MissionItemCurrentTopic &copyObj) :
    MissionItem::MissionItemCurrent()
{
    this->key = copyObj.getMissionKey();
    this->indexCurrent = copyObj.getMissionCurrentIndex();
}

} //end of namespace MissionTopic
