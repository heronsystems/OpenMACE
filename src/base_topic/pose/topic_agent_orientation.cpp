#include "topic_agent_orientation.h"

namespace pose {
namespace BaseTopic{

const char TopicName_AgentOrientation[] = "TOPIC_AGENTORIENTATION";

const MaceCore::TopicComponentStructure Structure_AgentOrientation = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Orientation Name");
    return structure;
}();

MaceCore::TopicDatagram Topic_AgentOrientation::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    return datagram;
}

void Topic_AgentOrientation::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    delete this->m_PositionObject; m_PositionObject = nullptr;

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {
        mace::pose::GeodeticPosition_2D* tmpObj = new mace::pose::GeodeticPosition_2D();
        tmpObj->setName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame"));

        Eigen::Vector2d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateTranslationalComponents(data(1),data(0));

        this->m_PositionObject = tmpObj;
    }
    else if(dimension == 3)
    {
        mace::pose::GeodeticPosition_3D* tmpObj = new mace::pose::GeodeticPosition_3D();
        tmpObj->setName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame"));
        tmpObj->setAltitudeReferenceFrame(datagram.GetTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame"));

        Eigen::Vector3d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updatePosition(data(1),data(0),data(2));

        this->m_PositionObject = tmpObj;
    }
}

Topic_AgentOrientation::Topic_AgentOrientation():
    m_PositionObject(nullptr)
{

}

Topic_AgentOrientation::Topic_AgentOrientation(const mace::pose::Abstract_GeodeticPosition *posObj)
{
    delete m_PositionObject; m_PositionObject = nullptr;
    //copy the contents of that point to the current pointer object
    m_PositionObject = posObj->getGeodeticClone();
}

Topic_AgentOrientation::Topic_AgentOrientation(const Topic_AgentOrientation &copy)
{
    this->m_PositionObject = copy.m_PositionObject->getGeodeticClone();
}

mace::pose::Abstract_GeodeticPosition* Topic_AgentOrientation::getPositionObj() const
{
    return this->m_PositionObject;
}

} //end of namespace BaseTopic
} //end of namespace pose
