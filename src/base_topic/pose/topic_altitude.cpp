#include "topic_altitude.h"

namespace pose{

namespace BaseTopic{

const char TopicName_Altitude[] = "TOPIC_ALTITUDE";

const MaceCore::TopicComponentStructure Structure_Altitude = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("Altitude Name");
    structure.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame");
    structure.AddTerminal<double>("Data");
    return structure;
}();

MaceCore::TopicDatagram Topic_Altitude::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("Altitude Name", altitudeObj.getName());
    datagram.AddTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame", altitudeObj.getAltitudeReferenceFrame());
    datagram.AddTerminal<double>("Data", altitudeObj.getAltitude());
    return datagram;
}

void Topic_Altitude::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {

    altitudeObj.setName(datagram.GetTerminal<std::string>("Altitude Name"));
    altitudeObj.setAltitudeReferenceFrame(datagram.GetTerminal<mace::AltitudeReferenceTypes>("Explicit Altitude Frame"));
    altitudeObj.setAltitude(datagram.GetTerminal<double>("Data"));
}

Topic_Altitude::Topic_Altitude()
{

}

Topic_Altitude::Topic_Altitude(const mace::pose::Altitude &altObj)
{
    //copy the contents of that point to the current pointer object
    this->altitudeObj = altObj;
}

Topic_Altitude::Topic_Altitude(const Topic_Altitude &copy)
{
    this->altitudeObj = copy.altitudeObj;
}

mace::pose::Altitude Topic_Altitude::getAltitudeObj() const
{
    return this->altitudeObj;
}

} //end of namespace BaseTopic
} //end of namespace pose
