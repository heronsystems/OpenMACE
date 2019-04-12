#include "mission_item_topic.h"
namespace MissionTopic{

const char MissionItemTopic_name[] = "missionItem";
const MaceCore::TopicComponentStructure MissionItemTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::shared_ptr<CommandItem::AbstractCommandItem>>("missionItem");
    return structure;
}();

MaceCore::TopicDatagram MissionItemTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::shared_ptr<CommandItem::AbstractCommandItem>>("missionItem", missionItem);
    return datagram;
}

void MissionItemTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
{
    missionItem = datagram.GetTerminal<std::shared_ptr<CommandItem::AbstractCommandItem>>("missionItem");
}


MissionItemTopic::MissionItemTopic()
{

}

void MissionItemTopic::setMissionItem(const std::shared_ptr<CommandItem::AbstractCommandItem> missionItem)
{
    this->missionItem = missionItem;
}

std::shared_ptr<CommandItem::AbstractCommandItem> MissionItemTopic::getMissionItem()
{
    return missionItem;
}

} //end of namespace MissionTopic
