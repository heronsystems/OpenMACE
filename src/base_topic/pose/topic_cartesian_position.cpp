#include "topic_cartesian_position.h"

namespace mace{

namespace pose_topics{

const char TopicName_CartesianPosition[] = "TOPIC_CARTESIANPOSITION";

const MaceCore::TopicComponentStructure Structure_CartesianPosition = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Position Name");
    structure.AddTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_CartesianPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("Position Name",positionObj->getName());
    datagram.AddTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame", positionObj->getCartesianCoordinateFrame());
    if(positionObj->is3D())
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", positionObj->positionAs<mace::pose::CartesianPosition_3D>()->getAltitudeReferenceFrame());
    else
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", AltitudeReferenceTypes::REF_ALT_UNKNOWN);

    datagram.AddTerminal<uint8_t>("Dimension", positionObj->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", positionObj->getDataVector());
    if(positionObj->is3D())
    {
        mace::AltitudeReferenceTypes altRef = positionObj->positionAs<mace::pose::CartesianPosition_3D>()->getAltitudeReferenceFrame();
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", altRef);
    }

    return datagram;
}

void Topic_CartesianPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    if(positionObj) {
        delete this->positionObj;
        positionObj = nullptr;
    }

    uint8_t dimension = datagram.GetTerminal<uint8_t>("Dimension");
    if(dimension == 2)
    {
        mace::pose::CartesianPosition_2D* tmpObj = new mace::pose::CartesianPosition_2D();
        tmpObj->setName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame"));

        Eigen::Vector2d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updatePosition(data(0),data(1));

        this->positionObj = tmpObj;
    }
    else if(dimension == 3)
    {
        mace::pose::CartesianPosition_3D* tmpObj = new mace::pose::CartesianPosition_3D();
        tmpObj->setName(datagram.GetTerminal<std::string>("Position Name"));
        tmpObj->setCoordinateFrame(datagram.GetTerminal<mace::CartesianFrameTypes>("Explicit Coordinate Frame"));
        tmpObj->setAltitudeReferenceFrame(datagram.GetTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame"));

        Eigen::Vector3d data = datagram.GetTerminal<Eigen::VectorXd>("Data");
        tmpObj->updatePosition(data(0),data(1),data(2));

        this->positionObj = tmpObj;
    }
}

Topic_CartesianPosition::Topic_CartesianPosition():
    positionObj(nullptr)
{

}

Topic_CartesianPosition::Topic_CartesianPosition(const mace::pose::Abstract_CartesianPosition *posObj)
{
    //copy the contents of that point to the current pointer object
    positionObj = posObj->getCartesianClone();
}

Topic_CartesianPosition::Topic_CartesianPosition(const Topic_CartesianPosition &copy)
{
    this->positionObj = copy.positionObj->getCartesianClone();
}

mace::pose::Abstract_CartesianPosition* Topic_CartesianPosition::getPositionObj() const
{
    return this->positionObj;
}

} //end of namespace BaseTopic
} //end of namespace pose
