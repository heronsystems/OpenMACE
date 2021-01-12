#include "topic_cartesian_velocity.h"

namespace mace{

namespace pose_topics{

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
    datagram.AddTerminal<std::string>("Object Name", m_VelocityObj->getName());
    datagram.AddTerminal<uint8_t>("Dimension", m_VelocityObj->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", m_VelocityObj->getDataVector());
    return datagram;
}

void Topic_CartesianVelocity::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    delete this->m_VelocityObj; m_VelocityObj = nullptr;

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {
        mace::pose::Velocity_Cartesian2D* tmpObj = new mace::pose::Velocity_Cartesian2D(mace::CartesianFrameTypes::CF_LOCAL_NED);
        tmpObj->updateVelocityName(datagram.GetTerminal<std::string>("Object Name"));
        Eigen::VectorXd data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateDataVector(data);

        this->m_VelocityObj = tmpObj;
    }
    else if(dimension == 3)
    {
        mace::pose::Velocity_Cartesian3D* tmpObj = new mace::pose::Velocity_Cartesian3D(mace::CartesianFrameTypes::CF_LOCAL_NED);
        tmpObj->updateVelocityName(datagram.GetTerminal<std::string>("Object Name"));
        Eigen::VectorXd data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateDataVector(data);

        this->m_VelocityObj = tmpObj;
    }
}

Topic_CartesianVelocity::Topic_CartesianVelocity():
    m_VelocityObj(nullptr)
{

}

Topic_CartesianVelocity::Topic_CartesianVelocity(const mace::pose::Velocity* velObj)
{
    //copy the contents of that point to the current pointer object
    m_VelocityObj = velObj->getVelocityClone();
}

Topic_CartesianVelocity::Topic_CartesianVelocity(const Topic_CartesianVelocity &copy)
{
    this->m_VelocityObj = copy.m_VelocityObj->getVelocityClone();
}

QJsonObject Topic_CartesianVelocity::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    mace::pose::Velocity* castVelocity = getVelocityObj()->velocityAs<mace::pose::Velocity_Cartesian3D>();
        json["xVel"] = castVelocity->getDataVector().x();
        json["yVel"] = castVelocity->getDataVector().y();
        json["zVel"] = castVelocity->getDataVector().z();
    return json;
}

void Topic_CartesianVelocity::fromJSON(const QJsonDocument &inputJSON)
{
    mace::pose::Velocity_Cartesian3D* tmpObj = getVelocityObj()->getVelocityClone()->velocityAs<mace::pose::Velocity_Cartesian3D>();
    tmpObj->setXVelocity(inputJSON.object().value("xVel").toDouble());
    tmpObj->setYVelocity(inputJSON.object().value("yVel").toDouble());
    tmpObj->setZVelocity(inputJSON.object().value("zVel").toDouble());
    this->m_VelocityObj = tmpObj;
}

std::string Topic_CartesianVelocity::toCSV(const std::string &delimiter) const
{
    const mace::pose::Velocity_Cartesian3D* castVelocity = getVelocityObj()->velocityAs<mace::pose::Velocity_Cartesian3D>();
    std::string newline = std::to_string(castVelocity->getXVelocity()) + delimiter + std::to_string(castVelocity->getYVelocity()) + delimiter + std::to_string(castVelocity->getZVelocity());
    return newline;
}

mace::pose::Velocity* Topic_CartesianVelocity::getVelocityObj() const
{
    return this->m_VelocityObj;
}

} //end of namespace BaseTopic
} //end of namespace pose
