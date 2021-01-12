#include "data_generic_item_topic_text.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicText_name[] = "statusText";
const MaceCore::TopicComponentStructure DataGenericItemTopicText_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<MAV_SEVERITY>("severity");
    structure.AddTerminal<std::string>("text");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_Text::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<MAV_SEVERITY>("severity", _severity);
    datagram.AddTerminal<std::string>("text", _dataString);
    return datagram;
}

void DataGenericItemTopic_Text::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    _severity = datagram.GetTerminal<MAV_SEVERITY>("severity");
    _dataString = datagram.GetTerminal<std::string>("text");
}

QJsonObject DataGenericItemTopic_Text::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["severity"] =  QString::fromStdString(DataGenericItem::DataGenericItem_Text::StatusSeverityToString(getSeverity()));
    json["text"] = QString::fromStdString(getText());
    return json;
}

void DataGenericItemTopic_Text::fromJSON(const QJsonDocument &inputJSON)
{
    this->setText(inputJSON.object().value("text").toString().toStdString());
}

std::string DataGenericItemTopic_Text::toCSV(const std::string &delimiter) const
{
    UNUSED(delimiter);
    std::string newline = getText();
    std::replace(newline.begin(), newline.end(), ',', '-');
    return newline;
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
