#include "data_generic_item_topic_system_arm.h"


namespace DataGenericItemTopic {

const char DataGenericItemTopicSystemArm_name[] = "systemArm";
const MaceCore::TopicComponentStructure DataGenericItemTopicSystemArm_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<bool>("armed");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_SystemArm::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<bool>("armed", armed);
    return datagram;
}

void DataGenericItemTopic_SystemArm::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    armed = datagram.GetTerminal<bool>("armed");
}

QJsonObject DataGenericItemTopic_SystemArm::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["armed"] = getSystemArm();
    return json;
}

DataGenericItemTopic_SystemArm::DataGenericItemTopic_SystemArm()
    :DataGenericItem::DataGenericItem_SystemArm()
{

}

DataGenericItemTopic_SystemArm::DataGenericItemTopic_SystemArm(const DataGenericItem::DataGenericItem_SystemArm &copyObj):
    DataGenericItem::DataGenericItem_SystemArm(copyObj)
{

}

} //end of namespace DataGenericItemTopic
