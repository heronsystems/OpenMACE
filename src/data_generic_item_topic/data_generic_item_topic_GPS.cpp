#include "data_generic_item_topic_GPS.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicGPS_name[] = "gpsStatus";
const MaceCore::TopicComponentStructure DataGenericItemTopicGPS_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<DataGenericItemTopic_GPS::GPSFixType>("fix");
    structure.AddTerminal<uint16_t>("satellitesVisible");
    structure.AddTerminal<uint16_t>("vdop");
    structure.AddTerminal<uint16_t>("hdop");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_GPS::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GPSFixType>("fix", fixtype);
    datagram.AddTerminal<uint16_t>("satellitesVisible", satellitesVisible);
    datagram.AddTerminal<uint16_t>("vdop", VDOP);
    datagram.AddTerminal<uint16_t>("hdop", HDOP);
    return datagram;
}

void DataGenericItemTopic_GPS::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    fixtype = datagram.GetTerminal<GPSFixType>("fix");
    satellitesVisible = datagram.GetTerminal<uint16_t>("satellitesVisible");
    VDOP = datagram.GetTerminal<uint16_t>("vdop");
    HDOP = datagram.GetTerminal<uint16_t>("hdop");
}

QJsonObject DataGenericItemTopic_GPS::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["visible_sats"] = getSatVisible();
    json["gps_fix"] = QString::fromStdString(DataGenericItem::DataGenericItem_GPS::GPSFixTypeToString(getGPSFix()));
    json["hdop"] = getHDOP();
    json["vdop"] = getVDOP();
    return json;
}

void DataGenericItemTopic_GPS::fromJSON(const std::string &inputJSON)
{
    //Pull Values from JSON string
    size_t s = inputJSON.find("visible_sats")+14;
    std::string sats = inputJSON.substr(s, inputJSON.find("}", s) - s );
    s = inputJSON.find("gps_fix")+10;
    std::string gps = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    s = inputJSON.find("hdop")+6;
    std::string hdop = inputJSON.substr(s, inputJSON.find(",", s) - s );
    s = inputJSON.find("vdop")+6;
    std::string vdop = inputJSON.substr(s, inputJSON.find(",", s) - s );

    this->setSatVisible(std::stod(sats));
    this->setGPSFix(GPSFixTypeFromString(gps));
    this->setHDOP(std::stod(hdop));
    this->setVDOP(std::stod(vdop));
}

std::string DataGenericItemTopic_GPS::toCSV() const
{
    std::string newline = std::to_string(getSatVisible()) + "; " + DataGenericItem::DataGenericItem_GPS::GPSFixTypeToString(getGPSFix()) + "; " + std::to_string(getHDOP()) + "; "+ std::to_string(getVDOP()) + "; ";
    return newline;
}

DataGenericItemTopic_GPS::DataGenericItemTopic_GPS()
    :DataGenericItem::DataGenericItem_GPS()
{

}

DataGenericItemTopic_GPS::DataGenericItemTopic_GPS(const DataGenericItem::DataGenericItem_GPS &copyObj):
    DataGenericItem::DataGenericItem_GPS(copyObj)
{

}

} //end of namespace DataGenericItemTopic
