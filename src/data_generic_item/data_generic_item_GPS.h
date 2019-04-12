#ifndef DATA_GENERIC_ITEM_GPS_H
#define DATA_GENERIC_ITEM_GPS_H

#include <stdint.h>
#include <iostream>
#include <stdexcept>

#include "mace.h"

namespace DataGenericItem {

class DataGenericItem_GPS
{
public:
    enum class GPSFixType : uint8_t
    {
        GPS_FIX_NONE=0, /* No GPS connected | */
        GPS_FIX_NO_FIX=1, /* No position information, GPS is connected | */
        GPS_FIX_2D_FIX=2, /* 2D position | */
        GPS_FIX_3D_FIX=3, /* 3D position | */
        GPS_FIX_DGPS=4, /* DGPS/SBAS aided 3D position | */
        GPS_FIX_RTK_FLOAT=5, /* RTK float, 3D position | */
        GPS_FIX_RTK_FIXED=6, /* RTK Fixed, 3D position | */
        GPS_FIX_STATIC=7, /* Static fixed, typically used for base stations | */
    };

    static std::string GPSFixTypeToString(const GPSFixType &gpsFix) {
        switch (gpsFix) {
        case GPSFixType::GPS_FIX_NONE:
            return "NO GPS";
        case GPSFixType::GPS_FIX_NO_FIX:
            return "GPS NO FIX";
        case GPSFixType::GPS_FIX_2D_FIX:
            return "GPS 2D";
        case GPSFixType::GPS_FIX_3D_FIX:
            return "GPS 3D";
        case GPSFixType::GPS_FIX_DGPS:
            return "GPS DGPS";
        case GPSFixType::GPS_FIX_RTK_FLOAT:
            return "GPS RTK FLOAT";
        case GPSFixType::GPS_FIX_RTK_FIXED:
            return "GPS RTK FIXED";
        case GPSFixType::GPS_FIX_STATIC:
            return "GPS STATIC";
        default:
            throw std::runtime_error("Unknown gps fix seen");
        }
    }

    static GPSFixType GPSFixTypeFromString(const std::string &str) {
        if(str == "NO GPS")
            return GPSFixType::GPS_FIX_NONE;
        if(str == "GPS NO FIX")
            return GPSFixType::GPS_FIX_NO_FIX;
        if(str == "GPS 2D")
            return GPSFixType::GPS_FIX_2D_FIX;
        if(str == "GPS 3D")
            return GPSFixType::GPS_FIX_3D_FIX;
        if(str == "GPS DGPS")
            return GPSFixType::GPS_FIX_DGPS;
        if(str == "GPS RTK FLOAT")
            return GPSFixType::GPS_FIX_RTK_FLOAT;
        if(str == "GPS RTK FIXED")
            return GPSFixType::GPS_FIX_RTK_FIXED;
        if(str == "GPS STATIC")
            return GPSFixType::GPS_FIX_STATIC;
        throw std::runtime_error("Unknown gps fix seen");
    }

public:
    DataGenericItem_GPS();

    DataGenericItem_GPS(const DataGenericItem_GPS &copyObj);

    DataGenericItem_GPS(const mace_gps_raw_int_t &copyObj);


    void setGPSFix(const GPSFixType &fix){
        this->fixtype = fix;
    }
    GPSFixType getGPSFix() const{
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
        if(this->getGPSFix() >= GPSFixType::GPS_FIX_3D_FIX) {
            return true;
        }
        return false;
    }

    mace_gps_raw_int_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_GPS &rhs)
    {
        this->fixtype = rhs.fixtype;
        this->satellitesVisible = rhs.satellitesVisible;
        this->HDOP = rhs.HDOP;
        this->VDOP = rhs.VDOP;
    }

    bool operator == (const DataGenericItem_GPS &rhs) {
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

    bool operator != (const DataGenericItem_GPS &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"GPS Status( FixType: "<<GPSFixTypeToString(fixtype)<<", Satellites Visible: "<<(int)satellitesVisible<<", HDOP: "<<(int)HDOP<<", VDOP: "<<(int)VDOP<<")";
        return out;
    }

protected:
    GPSFixType fixtype;
    uint16_t satellitesVisible;
    uint16_t HDOP;
    uint16_t VDOP;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_GPS_H
