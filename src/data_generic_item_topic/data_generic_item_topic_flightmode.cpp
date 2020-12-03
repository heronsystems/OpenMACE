#include "data_generic_item_topic_flightmode.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicFlightMode_name[] = "system_operating_mode";
const MaceCore::TopicComponentStructure DataGenericItemTopicFlightMode_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<std::string>("systemModeString");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_FlightMode::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<std::string>("systemModeString", flightModeString);
    return datagram;
}


void DataGenericItemTopic_FlightMode::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    flightModeString = datagram.GetTerminal<std::string>("systemModeString");
}
QJsonObject DataGenericItemTopic_FlightMode::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["mode"] = QString::fromStdString(getFlightModeString());
    return json;
}

void DataGenericItemTopic_FlightMode::fromJSON(const std::string &inputJSON)
{
    //Pull Values from JSON string
    size_t s = inputJSON.find("\"mode")+8;
    std::string mode = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    this->setFlightMode(mode);
}

std::string DataGenericItemTopic_FlightMode::toCSV() const
{
    std::string newline = getFlightModeString() + "; " ;
    return newline;
}

DataGenericItemTopic_FlightMode::DataGenericItemTopic_FlightMode()
    :DataGenericItem::DataGenericItem_FlightMode()
{

}

DataGenericItemTopic_FlightMode::DataGenericItemTopic_FlightMode(const DataGenericItem::DataGenericItem_FlightMode &copyObj):
    DataGenericItem::DataGenericItem_FlightMode(copyObj)
{

}

} //end of namespace DataGenericItemTopic
