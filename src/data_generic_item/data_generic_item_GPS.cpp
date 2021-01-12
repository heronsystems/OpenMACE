#include "data_generic_item_GPS.h"

namespace DataGenericItem {

DataGenericItem_GPS::DataGenericItem_GPS() :
    fixtype(GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS), satellitesVisible(0), HDOP(UINT16_MAX), VDOP(UINT16_MAX)
{

}

DataGenericItem_GPS::DataGenericItem_GPS(const DataGenericItem_GPS &copyObj)
{
    this->fixtype = copyObj.getGPSFix();
    this->satellitesVisible = copyObj.getSatVisible();
    this->HDOP = copyObj.getHDOP();
    this->VDOP = copyObj.getVDOP();
}

DataGenericItem_GPS::DataGenericItem_GPS(const mavlink_gps_raw_int_t &copyObj)
{
    this->fixtype = static_cast<GPS_FIX_TYPE>(copyObj.fix_type);
    this->satellitesVisible = copyObj.satellites_visible;
    this->HDOP = copyObj.eph;
    this->VDOP = copyObj.epv;
}

mavlink_gps_raw_int_t DataGenericItem_GPS::getMACECommsObject() const
{
    mavlink_gps_raw_int_t rtnObj;
    rtnObj.fix_type = (uint8_t)this->fixtype;
    rtnObj.satellites_visible = this->satellitesVisible;
    rtnObj.eph = this->HDOP;
    rtnObj.epv = this->VDOP;

    return rtnObj;
}

mavlink_message_t DataGenericItem_GPS::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps = getMACECommsObject();
    mavlink_msg_gps_raw_int_encode_chan(systemID,compID,chan,&msg,&gps);
    return msg;
}

} //end of namespace DataGenericItem
