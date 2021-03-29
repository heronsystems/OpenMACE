#include "mission_command_topic.h"

namespace DataVehicleCommands {

const char MissionCommandTopic_Name[] = "MissionCommandTopic";

const MaceCore::TopicComponentStructure MissionCommandTopic_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<AbstractMissionItem>("missionCommand");
    return structure;
}();

MaceCore::TopicDatagram MissionCommandTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::shared_ptr<AbstractMissionItem>>("missionCommand", m_MissionItem);
    return datagram;
}

void MissionCommandTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_MissionItem = datagram.GetTerminal<std::shared_ptr<AbstractMissionItem>>("missionCommand");
}

void MissionCommandTopic::setMissionItem(const std::shared_ptr<AbstractMissionItem> &missionItem)
{
    m_MissionItem = missionItem;
}

} //end of namespace DataVehicleCommands

