#include "topic_geodetic_position.h"

namespace BaseTopic{

const char TopicName_GeodeticPosition[] = "TOPIC_GEODETICPOSITION";

const MaceCore::TopicComponentStructure GlobalPositionTopic_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<uint8_t>("Dimension");
    structure.AddTerminal<Eigen::VectorXd>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_GeodeticPosition::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<mace::GeodeticFrameTypes>("Explicit Coordinate Frame", positionObj->getGeodeticCoordinateFrame()));
    datagram.AddTerminal<uint8_t>("Dimension", positionObj->getDimension());
    datagram.AddTerminal<Eigen::VectorXd>("Data", positionObj->);
    if(positionObj->is3D())
    {
        mace::AltitudeReferenceTypes altRef = positionObj->positionAs<mace::pose::GeodeticPosition_3D>()->getAltitudeReferenceFrame();
        datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", altRef);
    }

    return datagram;
}

void Topic_GeodeticPosition::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_CoordinateFrame = datagram.GetTerminal<Data::CoordinateFrameType>("CoordinateFrame");
    x = datagram.GetTerminal<double>("latitude");
    y = datagram.GetTerminal<double>("longitude");
    z = datagram.GetTerminal<double>("altitude");
}


Topic_GeodeticPosition::Topic_GeodeticPosition()
    :DataState::StateGlobalPosition()
{

}

Topic_GeodeticPosition::Topic_GeodeticPosition(const Topic_GeodeticPosition &copy)
{

}

Topic_GeodeticPosition::Topic_GeodeticPosition(const mace::pose::Abstract_GeodeticPosition *posObj):
    DataState::StateGlobalPosition(posObj)
{

}

} //end of namespace BaseTopic
