#include "position_global.h"

//namespace Data {

//namespace TopicComponents
//{

//const char TopicComponts_PositionGlobal_name[] = "position_global";
//const MaceCore::TopicComponentStructure TopicComponts_PositionGlobal_structure = []{
//    return MaceCore::TopicComponentStructure::Merge(TopicComponentPrototypes::TopicPrototype_Altitude_structure, TopicComponentPrototypes::PositionGeoreference_structure);
//}();


//MaceCore::TopicDatagram PositionGlobal::GenerateDatagram() const
//{
//    return MaceCore::TopicDatagram::Merge(TopicComponentPrototypes::Altitude::GenerateDatagram(), TopicComponentPrototypes::PositionGeoreference::GenerateDatagram());
//}


//void PositionGlobal::CreateFromDatagram(const MaceCore::TopicDatagram &datagram)
//{
//    TopicComponentPrototypes::Altitude::CreateFromDatagram(datagram);
//    TopicComponentPrototypes::PositionGeoreference::CreateFromDatagram(datagram);
//}

//PositionGlobal::PositionGlobal(const double &altitude, const ReferenceAltitude &alt_ref, const double &lattitude, const double &longitude, const ReferenceGeoCoords &geo_ref) :
//    TopicComponentPrototypes::Altitude(altitude, alt_ref),
//    TopicComponentPrototypes::PositionGeoreference(lattitude, longitude, geo_ref)
//{

//}

//PositionGlobal::PositionGlobal(const PositionGlobal &copyObj) :
//    TopicComponentPrototypes::Altitude(copyObj),
//    TopicComponentPrototypes::PositionGeoreference(copyObj)
//{

//}


//} // TopicComponents

//} // Data
