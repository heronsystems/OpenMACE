#include "topic_geodetic_position.h"

namespace BaseTopic{

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
    datagram.AddTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame", positionObj->getGeodeticCoordinateFrame());
    datagram.AddTerminal<uint8_t>("Dimension", positionObj->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", positionObj->getDataVector());
    if(positionObj->is3D())
    {
        mace::AltitudeReferenceTypes altRef = positionObj->positionAs<mace::pose::GeodeticPosition_3D>()->getAltitudeReferenceFrame();
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", altRef);
    }

    return datagram;
}

void Topic_GeodeticPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    delete this->positionObj; positionObj = nullptr;

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {
        mace::pose::GeodeticPosition_2D* tmpObj = new mace::pose::GeodeticPosition_2D();
        tmpObj->updatePositionName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame"));

        Eigen::Vector2d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updateTranslationalComponents(data(1),data(0));

        this->positionObj = tmpObj;
    }
    else if(dimension == 3)
    {
        mace::pose::GeodeticPosition_3D* tmpObj = new mace::pose::GeodeticPosition_3D();
        tmpObj->updatePositionName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame"));

        Eigen::Vector3d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updatePosition(data(1),data(0),data(2));

        this->positionObj = tmpObj;
    }
}

Topic_GeodeticPosition::Topic_GeodeticPosition():
    positionObj(nullptr)
{

}

Topic_GeodeticPosition::Topic_GeodeticPosition(const mace::pose::Abstract_GeodeticPosition *posObj):
{
    delete positionObj; positionObj = nullptr;
    //copy the contents of that point to the current pointer object
    positionObj = posObj->getGeodeticClone();
}

Topic_GeodeticPosition::Topic_GeodeticPosition(const Topic_GeodeticPosition &copy)
{
    this->positionObj = copy.positionObj->getGeodeticClone();
}

mace::pose::Abstract_GeodeticPosition* Topic_GeodeticPosition::getPositionObj() const
{
    return this->positionObj;
}

} //end of namespace BaseTopic
