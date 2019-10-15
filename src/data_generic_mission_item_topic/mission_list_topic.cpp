#include "mission_list_topic.h"

namespace MissionTopic{

const char MissionListTopic_name[] = "MissionList";
const MaceCore::TopicComponentStructure MissionListTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<unsigned int>("vehicleID");
    structure.AddTerminal<MissionItem::MissionList>("missionList");
    return structure;
}();

MaceCore::TopicDatagram MissionListTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<unsigned int>("vehicleID",vehicleID);
    datagram.AddTerminal<MissionItem::MissionList>("missionList", missionList);
    return datagram;
}

void MissionListTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    vehicleID = datagram.GetTerminal<unsigned int>("vehicleID");
    missionList = datagram.GetTerminal<MissionItem::MissionList>("missionList");
}

MissionListTopic::MissionListTopic()
{

}


MissionListTopic::MissionListTopic(const MissionItem::MissionList missionList)
{
    setMissionList(missionList);
}


void MissionListTopic::setMissionList(const MissionItem::MissionList missionListA)
{
    this->vehicleID = missionListA.getVehicleID();
    this->missionList = missionListA;
}

MissionItem::MissionList MissionListTopic::getMissionList()
{
    return missionList;
}

} //end of namespace MissionTopic
