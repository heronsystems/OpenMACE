#include "data_generic_item_topic_param_value.h"

namespace DataGenericItemTopic {

const char DataGenericItemTopicParamValue_name[] = "parameterValue";
const MaceCore::TopicComponentStructure DataGenericItemTopicParamValue_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<int>("indexID");
    structure.AddTerminal<std::string>("parameterID");
    structure.AddTerminal<double>("value");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_ParamValue::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<int>("indexID", _indexID);
    datagram.AddTerminal<std::string>("parameterID", _parameterID);
    datagram.AddTerminal<double>("value", _value);
    return datagram;
}

void DataGenericItemTopic_ParamValue::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    _indexID = datagram.GetTerminal<int>("indexID");
    _parameterID = datagram.GetTerminal<std::string>("parameterID");
    _value = datagram.GetTerminal<double>("value");
}

DataGenericItemTopic_ParamValue::DataGenericItemTopic_ParamValue()
    :DataGenericItem::DataGenericItem_ParamValue()
{

}

QJsonObject DataGenericItemTopic_ParamValue::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["indexID"] = _indexID;
    json["parameterID"] = QString::fromStdString(_parameterID);
    json["value"] = _value;
    return json;
}

DataGenericItemTopic_ParamValue::DataGenericItemTopic_ParamValue(const DataGenericItem::DataGenericItem_ParamValue &copyObj):
    DataGenericItem::DataGenericItem_ParamValue(copyObj)
{

}

} //end of namespace DataGenericItemTopic

