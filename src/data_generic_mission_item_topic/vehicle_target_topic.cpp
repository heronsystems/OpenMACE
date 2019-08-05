#include "vehicle_target_topic.h"

namespace MissionTopic{

const char VehicleTargetTopic_name[] = "VehicleTarget";
const MaceCore::TopicComponentStructure VehicleTargetTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("systemID");
    structure.AddTerminal<mace::pose::PositionPtr>("targetPosition");
    structure.AddTerminal<double>("targetDistance");
    structure.AddTerminal<Data::ControllerState>("targetState");
    return structure;
}();

MaceCore::TopicDatagram VehicleTargetTopic::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("systemID",systemID);
    datagram.AddTerminal<mace::pose::PositionPtr>("targetPosition", targetPosition);
    datagram.AddTerminal<double>("targetDistance", targetDistance);
    datagram.AddTerminal<Data::ControllerState>("targetState", targetState);
    return datagram;
}

void VehicleTargetTopic::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    systemID = datagram.GetTerminal<int>("systemID");
    targetPosition = datagram.GetTerminal<mace::pose::PositionPtr>("targetPosition");
    targetDistance = datagram.GetTerminal<double>("targetDistance");
    targetState = datagram.GetTerminal<Data::ControllerState>("targetState");
}

VehicleTargetTopic::VehicleTargetTopic() :
    systemID(0), targetDistance(0.0), targetState(Data::ControllerState::UNKNOWN)
{

}

VehicleTargetTopic::VehicleTargetTopic(const int &vehicleID, const mace::pose::PositionPtr position,
                                       const double &distance, const Data::ControllerState &state):
    systemID(vehicleID), targetPosition(position), targetDistance(distance), targetState(state)
{

}


VehicleTargetTopic::VehicleTargetTopic(const VehicleTargetTopic &copy)
{
    this->systemID = copy.systemID;
    this->targetPosition = copy.targetPosition;
    this->targetDistance = copy.targetDistance;
    this->targetState = copy.targetState;
}

VehicleTargetTopic::VehicleTargetTopic(const mace_guided_target_stats_t &obj)
{
    //Ken we need to reconstruct inside here
    this->targetDistance = static_cast<double>(obj.distance);
    this->targetState = static_cast<Data::ControllerState>(obj.state);
}

mace_guided_target_stats_t VehicleTargetTopic::getMACECommsObject() const
{
    //Ken we need to reconstruct inside here
    mace_guided_target_stats_t rtn;
//    rtn.coordinate_frame = (uint8_t)this->targetPosition.getCoordinateFrame();
//    rtn.distance = this->targetDistance;
//    rtn.x = this->targetPosition.getX();
//    rtn.y = this->targetPosition.getY();
//    rtn.z = this->targetPosition.getZ();
//    rtn.state = (uint8_t)this->targetState;
    return rtn;
}
mace_message_t VehicleTargetTopic::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_guided_target_stats_t target = getMACECommsObject();
    mace_message_t msg;
    mace_msg_guided_target_stats_encode_chan(systemID,compID,chan,&msg,&target);
    return msg;
}

std::ostream& operator<<(std::ostream& os, const VehicleTargetTopic& t)
{
    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Target Topic for system " << t.systemID << ": " << t.targetPosition.getX() << ", "<< t.targetPosition.getY() << ", "<< t.targetPosition.getZ() << ", "<< t.targetDistance<<", "<<Data::ControllerStateToString(t.targetState)<< ".";
    os << stream.str();

    return os;
}


} //end of namespace MissionTopic
