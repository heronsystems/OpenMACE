#include "position_cartesian_3d.h"

namespace Data {

namespace TopicComponentPrototypes
{

const MaceCore::TopicComponentStructure PositionCartesian3D_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<Data::ReferenceCartesian>("reference");
    structure.AddTerminal<double>("x");
    structure.AddTerminal<double>("y");
    structure.AddTerminal<double>("z");
    return structure;
}();

MaceCore::TopicDatagram PositionCartesian3D::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<Data::ReferenceCartesian>("reference", m_Reference);
    datagram.AddTerminal<double>("x", m_x);
    datagram.AddTerminal<double>("y", m_y);
    datagram.AddTerminal<double>("z", m_z);
    return datagram;
}

void PositionCartesian3D::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_Reference = datagram.GetTerminal<Data::ReferenceCartesian>("reference");
    m_x = datagram.GetTerminal<double>("x");
    m_y = datagram.GetTerminal<double>("y");
    m_z = datagram.GetTerminal<double>("z");
}


PositionCartesian3D::PositionCartesian3D(const PositionCartesian3D &copyObj) :
    PositionCartesian3D(copyObj.m_x, copyObj.m_y, copyObj.m_z, copyObj.m_Reference)
{

}

PositionCartesian3D::PositionCartesian3D(const double &x, const double &y, const double &z, const ReferenceCartesian &ref) :
    m_x(x),
    m_y(y),
    m_z(z),
    m_Reference(ref)
{

}

} // TopicComponentPrototypes

} // Data
