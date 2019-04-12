#include "command_topic_land.h"

namespace DataCommandTopic {

const char DataCommandTopicLand_name[] = "command_land";
const MaceCore::TopicComponentStructure DataCommandTopicLand_structure = []{
    MaceCore::TopicComponentStructure structure = DataStateTopic::PrototypeTopicGlobalPosition_structure;
    structure.AddTerminal<bool>("positionSet");
    return structure;
}();

MaceCore::TopicDatagram DataCommandTopic_Land::GenerateDatagram() const {
    if(m_PositionSet == true)
    {
        MaceCore::TopicDatagram datagram = m_position.Datagram();
        datagram.AddTerminal("positionSet", true);
    }
    else {
        MaceCore::TopicDatagram datagram;
        datagram.AddTerminal("positionSet", false);
    }
}

void DataCommandTopic_Land::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    if(datagram.GetTerminal<bool>("positionSet") == true)
    {
        m_position.CreateFromDatagram(datagram);
        m_PositionSet = true;
    }
    else {
        m_PositionSet = false;
    }
}

DataCommandTopic_Land::DataCommandTopic_Land() :
    m_PositionSet(false),
    m_position()
{

}

DataCommandTopic_Land::DataCommandTopic_Land(const DataState::StateGlobalPosition &copyObj) :
    m_PositionSet(true),
    m_position(copyObj)
{

}

}
