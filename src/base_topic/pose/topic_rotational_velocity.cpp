#include "topic_rotational_velocity.h"

namespace mace{

namespace pose_topics{

const char TopicName_RotationalVelocity[] = "TOPIC_ROTATIONALVELOCITY";

const MaceCore::TopicComponentStructure Structure_RotationalVelocity = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Object Name");
    structure.AddTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_RotationalVelocity::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("Object Name", m_VelocityObj.getName());
    datagram.AddTerminal<uint8_t>("Dimension", m_VelocityObj.getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", m_VelocityObj.getDataVector());
    return datagram;
}

void Topic_RotationalVelocity::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {

    }
    else if(dimension == 3)
    {
        mace::pose::Velocity_Rotation3D tmpObj;
        tmpObj.updateVelocityName(datagram.GetTerminal<std::string>("Object Name"));
        Eigen::VectorXd data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj.updateDataVector(data);

        this->m_VelocityObj = tmpObj;
    }
}

Topic_RotationalVelocity::Topic_RotationalVelocity()
{

}

Topic_RotationalVelocity::Topic_RotationalVelocity(const mace::pose::Velocity_Rotation3D &velObj)
{
    //copy the contents
    m_VelocityObj = velObj;
}

Topic_RotationalVelocity::Topic_RotationalVelocity(const Topic_RotationalVelocity &copy)
{
    this->m_VelocityObj = copy.m_VelocityObj;
}

mace::pose::Velocity_Rotation3D Topic_RotationalVelocity::getVelocityObj() const
{
    return this->m_VelocityObj;
}

} //end of namespace BaseTopic
} //end of namespace pose
