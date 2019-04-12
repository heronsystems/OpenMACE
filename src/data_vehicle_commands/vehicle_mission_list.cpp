#include "vehicle_mission_list.h"

namespace DataVehicleCommands {

const char VehicleMissionList_Name[] = "VehicleMissionList";

const MaceCore::TopicComponentStructure VehicleMissionList_Structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<CommandTypes>("missionType");
    structure.AddTerminal<std::string>("missionName");
    structure.AddTerminal<double>("missionLength");
    structure.AddTerminal<double>("missionDuration");
    structure.AddTerminal<std::list<AbstractMissionItem*>>("missionList");
    return structure;
}();

MaceCore::TopicDatagram VehicleMissionList::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<CommandTypes>("missionType",missionType);
    datagram.AddTerminal<std::string>("missionName", missionName);
    datagram.AddTerminal<double>("missionLength", missionLength);
    datagram.AddTerminal<double>("missionDuration", missionDuration);
    datagram.AddTerminal<std::list<AbstractMissionItem*>>("missionList", missionList);
    return datagram;
}

void VehicleMissionList::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    missionType = datagram.GetTerminal<CommandTypes>("missionType");
    missionName = datagram.GetTerminal<std::string>("missionName");
    missionLength = datagram.GetTerminal<double>("missionLength");
    missionDuration = datagram.GetTerminal<double>("missionDuration");
    missionList = datagram.GetTerminal<std::list<AbstractMissionItem*>>("missionList");
}

VehicleMissionList::VehicleMissionList():
    missionType(CommandTypes::MISSION), missionName(""),missionLength(0.0),missionDuration(0.0)
{

}

void VehicleMissionList::setMissionType(const CommandTypes &missionType)
{
    this->missionType = missionType;
}

CommandTypes VehicleMissionList::getMissionType()
{
    return missionType;
}

//void VehicleMissionList::setMissionList(const std::shared_ptr<std::list<AbstractMissionCommand>> &missionList)
//{
//    this->missionList = missionList;
//}

std::list<AbstractMissionItem*> VehicleMissionList::getMissionList()
{
    return missionList;
}

void VehicleMissionList::appendCommand(AbstractMissionItem *missionCommand)
{
    missionList.push_back(missionCommand);
}

void VehicleMissionList::clearCommands()
{
    missionList.clear();
}

void VehicleMissionList::removeCommand(const int &index)
{

}

}
