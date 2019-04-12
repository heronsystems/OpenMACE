#include "data_generic_item_topic_systemtime.h"


namespace DataGenericItemTopic {

const char DataGenericItemTopicSystemTime_name[] = "systemTime";
const MaceCore::TopicComponentStructure DataGenericItemTopicSystemTime_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("usec_since_epoch");
    structure.AddTerminal<double>("ms_since_system_boot");
    return structure;
}();

MaceCore::TopicDatagram DataGenericItemTopic_SystemTime::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    datagram.AddTerminal<uint64_t>("usec_since_epoch", usec_since_epoch);
    datagram.AddTerminal<uint32_t>("ms_since_system_boot", ms_since_system_boot);
    return datagram;
}

void DataGenericItemTopic_SystemTime::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    usec_since_epoch = datagram.GetTerminal<uint64_t>("usec_since_epoch");
    ms_since_system_boot = datagram.GetTerminal<uint32_t>("ms_since_system_boot");
}

DataGenericItemTopic_SystemTime::DataGenericItemTopic_SystemTime()
    :DataGenericItem::DataGenericItem_SystemTime()
{

}

DataGenericItemTopic_SystemTime::DataGenericItemTopic_SystemTime(const DataGenericItem::DataGenericItem_SystemTime &copyObj):
    DataGenericItem::DataGenericItem_SystemTime(copyObj)
{

}

} //end of namespace DataGenericItemTopic
