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
    datagram.AddTerminal<double>("voltage", voltage);
    datagram.AddTerminal<double>("current", current);
    datagram.AddTerminal<double>("remaining", batteryRemaing);
    return datagram;
}

void DataGenericItemTopic_Battery::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    voltage = datagram.GetTerminal<double>("voltage");
    current = datagram.GetTerminal<double>("current");
    batteryRemaing = datagram.GetTerminal<double>("remaining");
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

DataGenericItemTopic_Battery::DataGenericItemTopic_Battery(const DataGenericItem::DataGenericItem_Battery &copyObj):
    DataGenericItem::DataGenericItem_Battery(copyObj)
{

}

} //end of namespace DataGenericItemTopic
