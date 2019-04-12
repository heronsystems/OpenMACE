#include "position_georeference.h"

namespace Data {

namespace TopicComponentPrototypes
{

const MaceCore::TopicComponentStructure PositionGeoreference_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::ReferenceGeoCoords>("Frame");
    structure.AddTerminal<double>("lattitude");
    structure.AddTerminal<double>("longitude");
    return structure;
}();

MaceCore::TopicDatagram PositionGeoreference::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::ReferenceGeoCoords>("Frame", m_Reference);
    datagram.AddTerminal<double>("lattitude", m_lattitude);
    datagram.AddTerminal<double>("longitude", m_longitude);
    return datagram;
}

void PositionGeoreference::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_Reference = datagram.GetTerminal<Data::ReferenceGeoCoords>("Frame");
    m_lattitude = datagram.GetTerminal<double>("lattitude");
    m_longitude = datagram.GetTerminal<double>("longitude");
}


PositionGeoreference::PositionGeoreference(const PositionGeoreference &copyObj) :
    PositionGeoreference(copyObj.m_lattitude, copyObj.m_longitude, copyObj.m_Reference)
{

}

PositionGeoreference::PositionGeoreference(const double &lattitude, const double &longitude, const ReferenceGeoCoords &ref) :
    m_lattitude(lattitude),
    m_longitude(longitude),
    m_Reference(ref)
{

}

} // TopicComponentPrototypes

} // Data
