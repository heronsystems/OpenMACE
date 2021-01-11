#include "data_generic_item_topic_battery.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicBattery_name[] = "battery";
const MaceCore::TopicComponentStructure DataGenericItemTopicBattery_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("voltage");
    structure.AddTerminal<double>("current");
    structure.AddTerminal<double>("remaining");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Battery::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<double>("voltage", _voltage);
    datagram.AddTerminal<double>("current", _current);
    datagram.AddTerminal<double>("remaining", _batteryRemaing);
    return datagram;
}

void DataGenericItemTopic_Battery::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    _voltage = datagram.GetTerminal<double>("voltage");
    _current = datagram.GetTerminal<double>("current");
    _batteryRemaing = datagram.GetTerminal<double>("remaining");
}

DataGenericItemTopic_Battery::DataGenericItemTopic_Battery()
    :DataGenericItem::DataGenericItem_Battery()
{

}

QJsonObject DataGenericItemTopic_Battery::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["battery_remaining"] = getBatteryRemaining();
    json["battery_current"] = getBatteryCurrent();
    json["battery_voltage"] = getBatteryVoltage();
    return json;
}

void DataGenericItemTopic_Battery::fromJSON(const QJsonDocument &inputJSON)
{
    setBatteryRemaining(inputJSON.object().value("battery_remaining").toDouble());
    setBatteryCurrent(inputJSON.object().value("battery_current").toDouble());
    setBatteryVoltage(inputJSON.object().value("battery_voltage").toDouble());
}

std::string DataGenericItemTopic_Battery::toCSV(const std::string &delimiter) const
{
    std::string newline = std::to_string(getBatteryRemaining()) + delimiter + std::to_string(getBatteryCurrent()) + delimiter + std::to_string(getBatteryVoltage());
    return newline;
}
DataGenericItemTopic_Battery::DataGenericItemTopic_Battery(const DataGenericItem::DataGenericItem_Battery &copyObj):
    DataGenericItem::DataGenericItem_Battery(copyObj)
{

}

} //end of namespace DataGenericItemTopic
