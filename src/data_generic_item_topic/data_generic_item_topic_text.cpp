#include "data_generic_item_topic_text.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicText_name[] = "statusText";
const MaceCore::TopicComponentStructure DataGenericItemTopicText_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<DataGenericItemTopic_Text::STATUS_SEVERITY>("severity");
    structure.AddTerminal<std::string>("text");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Text::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<STATUS_SEVERITY>("severity", severity);
    datagram.AddTerminal<std::string>("text", dataString);
    return datagram;
}

void DataGenericItemTopic_Text::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    severity = datagram.GetTerminal<STATUS_SEVERITY>("severity");
    dataString = datagram.GetTerminal<std::string>("text");
}

QJsonObject DataGenericItemTopic_Text::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["severity"] =  QString::fromStdString(DataGenericItem::DataGenericItem_Text::StatusSeverityToString(getSeverity()));
    json["text"] = QString::fromStdString(getText());
    return json;
}

DataGenericItemTopic_Text::DataGenericItemTopic_Text()
    :DataGenericItem::DataGenericItem_Text()
{

}

DataGenericItemTopic_Text::DataGenericItemTopic_Text(const DataGenericItem::DataGenericItem_Text &copyObj):
    DataGenericItem::DataGenericItem_Text(copyObj)
{

}

} //end of namespace DataGenericItemTopic
