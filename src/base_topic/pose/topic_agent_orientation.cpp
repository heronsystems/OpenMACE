#include "topic_agent_orientation.h"

namespace mace {
namespace pose_topics{

const char TopicName_AgentOrientation[] = "TOPIC_AGENTORIENTATION";

const MaceCore::TopicComponentStructure Structure_AgentOrientation = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Orientation Name");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::Quaterniond>("2DRotation");
    return structure;
}();

MaceCore::TopicDatagram Topic_AgentOrientation::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    return datagram;
}

void Topic_AgentOrientation::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    delete this->m_RotationObj; m_RotationObj = nullptr;

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");

    if(dimension == mace::pose::Rotation_2D::rotationalDOF)
    {
        mace::pose::Rotation_2D* tmpObj = new mace::pose::Rotation_2D();
        tmpObj->setObjectName(datagram.GetTerminal<std::string>("Orientation Name"));

        this->m_RotationObj = tmpObj;
    }
    else if(dimension == mace::pose::Rotation_3D::rotationalDOF)
    {
        mace::pose::Rotation_3D* tmpObj = new mace::pose::Rotation_3D();
        tmpObj->setObjectName(datagram.GetTerminal<std::string>("Orientation Name"));

        this->m_RotationObj = tmpObj;
    }
}

Topic_AgentOrientation::Topic_AgentOrientation():
    m_RotationObj(nullptr)
{

}

Topic_AgentOrientation::Topic_AgentOrientation(const mace::pose::AbstractRotation *obj)
{
    delete m_RotationObj; m_RotationObj = nullptr;
    //copy the contents of that point to the current pointer object
    m_RotationObj = obj->getRotationalClone();
}

Topic_AgentOrientation::Topic_AgentOrientation(const Topic_AgentOrientation &copy)
{
    this->m_RotationObj = copy.m_RotationObj->getRotationalClone();
}

mace::pose::AbstractRotation* Topic_AgentOrientation::getRotationObj() const
{
    return this->m_RotationObj;
}

} //end of namespace BaseTopic
} //end of namespace pose
