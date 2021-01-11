#include "topic_geodetic_position.h"

namespace mace {
namespace pose_topics{

const char TopicName_GeodeticPosition[] = "TOPIC_GEODETICPOSITION";

const MaceCore::TopicComponentStructure Structure_GeodeticPosition = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Position Name");
    structure.AddTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_GeodeticPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("Position Name", m_PositionObject->getName());
    datagram.AddTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame", m_PositionObject->getGeodeticCoordinateFrame());

    if(m_PositionObject->is3D())
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", m_PositionObject->positionAs<mace::pose::GeodeticPosition_3D>()->getAltitudeReferenceFrame());
    else
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", AltitudeReferenceTypes::REF_ALT_UNKNOWN);

    datagram.AddTerminal<uint8_t>("Dimension", m_PositionObject->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", m_PositionObject->getDataVector());
    return datagram;
}

void Topic_GeodeticPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

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

QJsonObject Topic_GeodeticPosition::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    const mace::pose::GeodeticPosition_3D* castPosition = getPositionObj()->positionAs<mace::pose::GeodeticPosition_3D>();
    json["lat"] = castPosition->getLatitude();
    json["lng"] = castPosition->getLongitude();
    json["alt"] = castPosition->getAltitude();
    return json;
}

void Topic_GeodeticPosition::fromJSON(const QJsonDocument &inputJSON)
{
    mace::pose::GeodeticPosition_3D* tmpObj = m_PositionObject->getGeodeticClone()->positionAs<mace::pose::GeodeticPosition_3D>();
    tmpObj->updatePosition(inputJSON.object().value("lat").toDouble(), inputJSON.object().value("lng").toDouble(), inputJSON.object().value("alt").toDouble());
    this->m_PositionObject = tmpObj;
}

std::string Topic_GeodeticPosition::toCSV(const std::string &delimiter) const
{
    const mace::pose::GeodeticPosition_3D* castPosition = getPositionObj()->positionAs<mace::pose::GeodeticPosition_3D>();
    std::string newline = std::to_string(castPosition->getLatitude()) + delimiter + std::to_string(castPosition->getLongitude()) + delimiter + std::to_string(castPosition->getAltitude());
    return newline;
}

Topic_GeodeticPosition::Topic_GeodeticPosition():
    m_PositionObject(nullptr)
{
}


Topic_GeodeticPosition::Topic_GeodeticPosition(const mace::pose::Abstract_GeodeticPosition *posObj)
{
    //copy the contents of that point to the current pointer object
    m_PositionObject = posObj->getGeodeticClone();
}

Topic_GeodeticPosition::Topic_GeodeticPosition(const Topic_GeodeticPosition &copy)
{
    this->m_PositionObject = copy.m_PositionObject->getGeodeticClone();
}

mace::pose::Abstract_GeodeticPosition* Topic_GeodeticPosition::getPositionObj() const
{
    return this->m_PositionObject;
}

} //end of namespace BaseTopic
} //end of namespace pose

