#include "topic_cartesian_velocity.h"

namespace pose{

using namespace BaseTopic;

const char TopicName_CartesianVelocity[] = "TOPIC_CARTESIANVELOCITY";

const MaceCore::TopicComponentStructure Structure_CartesianVelocity = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Object Name");
    structure.AddTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_CartesianVelocity::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<uint8_t>("Dimension", m_VelocityObj->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", m_VelocityObj->getDataVector());
    return datagram;
}

void Topic_CartesianVelocity::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    delete this->m_VelocityObj; m_VelocityObj = nullptr;

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {
        mace::pose::Cartesian_Velocity2D* tmpObj = new mace::pose::Cartesian_Velocity2D(mace::CartesianFrameTypes::CF_BODY_ENU);
        tmpObj->updateVelocityName(datagram.GetTerminal<std::string>("Object Name"));
        Eigen::Vector2d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateDataVector(data);

        this->m_VelocityObj = tmpObj;
    }
    else if(dimension == 3)
    {
        mace::pose::Cartesian_Velocity3D* tmpObj = new mace::pose::Cartesian_Velocity3D(mace::CartesianFrameTypes::CF_BODY_ENU);
        tmpObj->updateVelocityName(datagram.GetTerminal<std::string>("Object Name"));
        Eigen::Vector3d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateDataVector(data);

        this->m_VelocityObj = tmpObj;
    }
}

Topic_CartesianVelocity::Topic_CartesianVelocity():
    m_VelocityObj(nullptr)
{

}

Topic_CartesianVelocity::Topic_CartesianVelocity(const mace::pose::Abstract_Velocity* velObj)
{
    delete m_VelocityObj; m_VelocityObj = nullptr;
    //copy the contents of that point to the current pointer object
    m_VelocityObj = velObj->getVelocityClone();
}

Topic_CartesianVelocity::Topic_CartesianVelocity(const Topic_CartesianVelocity &copy)
{
    this->m_VelocityObj = copy.m_VelocityObj->getVelocityClone();
}

mace::pose::Abstract_Velocity* Topic_CartesianVelocity::getVelocityObj() const
{
    return this->m_VelocityObj;
}

} //end of namespace pose
