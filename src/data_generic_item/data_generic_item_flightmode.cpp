#include "data_generic_item_flightmode.h"

namespace DataGenericItem {

DataGenericItem_FlightMode::DataGenericItem_FlightMode():
    flightModeString("")
{

}

DataGenericItem_FlightMode::DataGenericItem_FlightMode(const std::string &mode)
{
    this->flightModeString = mode;
}


DataGenericItem_FlightMode::DataGenericItem_FlightMode(const DataGenericItem_FlightMode &copyObj)
{
    this->flightModeString = copyObj.getFlightModeString();
}

DataGenericItem_FlightMode::DataGenericItem_FlightMode(const mace_vehicle_mode_t &copyObj)
{
    this->flightModeString = copyObj.vehicle_mode;
}

mace_vehicle_mode_t DataGenericItem_FlightMode::getMACECommsObject() const
{
    mace_vehicle_mode_t rtnObj;

    strcpy(rtnObj.vehicle_mode,this->flightModeString.c_str());

    return rtnObj;
}

mace_message_t DataGenericItem_FlightMode::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_vehicle_mode_t mode = getMACECommsObject();
    mace_msg_vehicle_mode_encode_chan(systemID,compID,chan,&msg,&mode);
    return msg;
}

} //end of namespace DataGenericItem
