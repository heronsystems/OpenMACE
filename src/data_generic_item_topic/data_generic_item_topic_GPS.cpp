#include "data_generic_item_topic_GPS.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicGPS_name[] = "gpsStatus";
const MaceCore::TopicComponentStructure DataGenericItemTopicGPS_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<GPS_FIX_TYPE>("fix");
    structure.AddTerminal<uint16_t>("satellitesVisible");
    structure.AddTerminal<uint16_t>("vdop");
    structure.AddTerminal<uint16_t>("hdop");
    return structure;
}();


MaceCore::TopicDatagram DataGenericItemTopic_GPS::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<GPS_FIX_TYPE>("fix", fixtype);
    datagram.AddTerminal<uint16_t>("satellitesVisible", satellitesVisible);
    datagram.AddTerminal<uint16_t>("vdop", VDOP);
    datagram.AddTerminal<uint16_t>("hdop", HDOP);
    return datagram;
}

void DataGenericItemTopic_GPS::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    fixtype = datagram.GetTerminal<GPS_FIX_TYPE>("fix");
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

void DataGenericItemTopic_GPS::fromJSON(const QJsonDocument &inputJSON)
{
    this->setSatVisible(inputJSON.object().value("visible_sats").toInt());
    this->setGPSFix(GPSFixTypeFromString(inputJSON.object().value("gps_fix").toString().toStdString()));
    this->setHDOP(inputJSON.object().value("hdop").toInt());
    this->setVDOP(inputJSON.object().value("vdop").toInt());
}

std::string DataGenericItemTopic_GPS::toCSV(const std::string &delimiter) const
{
    std::string newline = std::to_string(getSatVisible()) + delimiter + DataGenericItem::DataGenericItem_GPS::GPSFixTypeToString(getGPSFix()) + delimiter + std::to_string(getHDOP()) + delimiter + std::to_string(getVDOP());
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
