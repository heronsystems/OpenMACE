#include "altitude.h"


//namespace Data {

//namespace TopicComponentPrototypes
//{

//const MaceCore::TopicComponentStructure TopicPrototype_Altitude_structure = []{
//    MaceCore::TopicComponentStructure structure;
//    structure.AddTerminal<Data::ReferenceAltitude>("AltitudeFrame");
//    structure.AddTerminal<double>("altitude");
//    return structure;
//}();

//MaceCore::TopicDatagram Altitude::GenerateDatagram() const {
//    MaceCore::TopicDatagram datagram;
//    datagram.AddTerminal<Data::ReferenceAltitude>("AltitudeFrame", m_Reference);
//    datagram.AddTerminal<double>("altitude", m_Altitude);
//    return datagram;
//}

//void Altitude::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
//    m_Reference = datagram.GetTerminal<Data::ReferenceAltitude>("AltitudeFrame");
//    m_Altitude = datagram.GetTerminal<double>("altitude");
//}


//Altitude::Altitude(const Altitude &copyObj) :
//    Altitude(copyObj.m_Altitude, copyObj.m_Reference)
//{

//}

//Altitude::Altitude(const double &altitude, const ReferenceAltitude &ref) :
//    m_Altitude(altitude),
//    m_Reference(ref)
//{

//}


//} // TopicComponentPrototypes

//} // Data
