#ifndef DATA_GENERIC_ITEM_GPS_H
#define DATA_GENERIC_ITEM_GPS_H

#include <stdint.h>
#include <iostream>
#include <stdexcept>

#include "mavlink.h"

namespace DataGenericItem {

class DataGenericItem_GPS
{
public:

    static std::string GPSFixTypeToString(const GPS_FIX_TYPE &gpsFix) {
        switch (gpsFix) {
        case GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS:
            return "NO GPS";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_NO_FIX:
            return "GPS NO FIX";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_2D_FIX:
            return "GPS 2D";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_3D_FIX:
            return "GPS 3D";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_DGPS:
            return "GPS DGPS";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_RTK_FLOAT:
            return "GPS RTK FLOAT";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_RTK_FIXED:
            return "GPS RTK FIXED";
        case GPS_FIX_TYPE::GPS_FIX_TYPE_STATIC:
            return "GPS STATIC";
        default:
            throw std::runtime_error("Unknown gps fix seen");
        }
    }

    static GPS_FIX_TYPE GPSFixTypeFromString(const std::string &str) {
        if(str == "NO GPS")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_NO_GPS;
        if(str == "GPS NO FIX")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_NO_FIX;
        if(str == "GPS 2D")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_2D_FIX;
        if(str == "GPS 3D")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_3D_FIX;
        if(str == "GPS DGPS")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_DGPS;
        if(str == "GPS RTK FLOAT")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_RTK_FLOAT;
        if(str == "GPS RTK FIXED")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_RTK_FIXED;
        if(str == "GPS STATIC")
            return GPS_FIX_TYPE::GPS_FIX_TYPE_STATIC;
        throw std::runtime_error("Unknown gps fix seen");
    }

public:
    DataGenericItem_GPS();

    DataGenericItem_GPS(const DataGenericItem_GPS &copyObj);

    DataGenericItem_GPS(const mavlink_gps_raw_int_t &copyObj);


    void setGPSFix(const GPS_FIX_TYPE &fix){
        this->fixtype = fix;
    }

    GPS_FIX_TYPE getGPSFix() const{
        return fixtype;
    }

    void setSatVisible(const uint16_t &satsVisible){
        this->satellitesVisible = satsVisible;
    }
    uint16_t getSatVisible() const{
        return satellitesVisible;
    }

    void setHDOP(const uint16_t &hdop){
        this->HDOP = hdop;
    }
    uint16_t getHDOP() const{
        return HDOP;
    }

    void setVDOP(const uint16_t &vdop){
        this->VDOP = vdop;
    }
    uint16_t getVDOP() const{
        return VDOP;
    }

    bool is3DorGreater() {
        if(this->getGPSFix() >= GPS_FIX_TYPE::GPS_FIX_TYPE_3D_FIX) {
            return true;
        }
        return false;
    }

    mavlink_gps_raw_int_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_GPS &rhs)
    {
        this->fixtype = rhs.fixtype;
        this->satellitesVisible = rhs.satellitesVisible;
        this->HDOP = rhs.HDOP;
        this->VDOP = rhs.VDOP;
    }

    bool operator == (const DataGenericItem_GPS &rhs) const {
        if(this->fixtype != rhs.fixtype){
            return false;
        }
        if(this->satellitesVisible != rhs.satellitesVisible){
            return false;
        }
        if(this->HDOP != rhs.HDOP){
            return false;
        }
        if(this->VDOP != rhs.VDOP){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_GPS &rhs) const{
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"GPS Status( FixType: "<<GPSFixTypeToString(fixtype)<<", Satellites Visible: "<<(int)satellitesVisible<<", HDOP: "<<(int)HDOP<<", VDOP: "<<(int)VDOP<<")";
        return out;
    }

protected:
    GPS_FIX_TYPE fixtype;
    uint16_t satellitesVisible;
    uint16_t HDOP;
    uint16_t VDOP;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_GPS_H
