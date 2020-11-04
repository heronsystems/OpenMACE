#ifndef DATA_GENERIC_ITEM_TOPIC_COMPONENTS_H
#define DATA_GENERIC_ITEM_TOPIC_COMPONENTS_H

#define DATA_GENERIC_VEHICLE_ITEM_TOPICS DataGenericItemTopic::DataGenericItemTopic_Heartbeat,\
    DataGenericItemTopic::DataGenericItemTopic_SystemArm, DataGenericItemTopic::DataGenericItemTopic_FlightMode,\
    DataGenericItemTopic::DataGenericItemTopic_Battery, DataGenericItemTopic::DataGenericItemTopic_GPS,\
    DataGenericItemTopic::DataGenericItemTopic_ParamValue,\
    DataGenericItemTopic::DataGenericItemTopic_Text, DataGenericItemTopic::DataGenericItemTopic_SystemTime

#include "data_generic_item_topic_battery.h"
#include "data_generic_item_topic_flightmode.h"
#include "data_generic_item_topic_GPS.h"
#include "data_generic_item_topic_heartbeat.h"
#include "data_generic_item_topic_param_value.h"
#include "data_generic_item_topic_system_arm.h"
#include "data_generic_item_topic_systemtime.h"
#include "data_generic_item_topic_text.h"

#endif // DATA_GENERIC_ITEM_TOPIC_COMPONENTS_H
