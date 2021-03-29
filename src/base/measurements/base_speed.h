#ifndef BASE_SPEED_H
#define BASE_SPEED_H

#include "mavlink.h"

#include <string>
#include <limits>
#include <cmath>

namespace mace {
namespace measurements {

class Speed
{
public:
    Speed();

    Speed(const Speed &copy);

    void updateSpeedMeasurement(const double &value);

    void updateSpeedType(const SPEED_TYPE &type);

    double getSpeedMeasurement() const;

    SPEED_TYPE getSpeedType() const;


    /**
     * @brief getClone
     * @return
     */
    Speed* getSpeedClone() const
    {
        return new Speed(*this);
    }

    /**
     * @brief getClone
     * @param state
     */
    void getSpeedClone(Speed** speed) const
    {
        *speed = new Speed(*this);
    }

public:
    //!
    //! \brief setSpeed
    //! \param speed
    //!
    void setSpeed(const double &speed);

    //!
    //! \brief getSpeed
    //! \return
    //!
    double getSpeed() const;

    mavlink_vfr_hud_t getMAVLINKOBJ() const
    {
        mavlink_vfr_hud_t obj;
        obj.airspeed = 0.0;
        obj.groundspeed = 0.0;
        switch (speedType) {
        case SPEED_TYPE::SPEED_TYPE_AIRSPEED:
        {
            obj.airspeed = static_cast<float>(measurement);
            break;
        }
        case SPEED_TYPE::SPEED_TYPE_GROUNDSPEED:
        {
            obj.groundspeed =static_cast<float>(measurement);
            break;
        }
        default:
            break;
        }
        return obj;
    }

    void fromMAVLINKMSG(const mavlink_vfr_hud_t &msg)
    {
        measurement = static_cast<double>(msg.airspeed);
    }

    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
    {
        mavlink_message_t msg;
        mavlink_vfr_hud_t obj = getMAVLINKOBJ();
        mavlink_msg_vfr_hud_encode_chan(systemID,compID,chan,&msg,&obj);
        return msg;
    }


public:
    void operator = (const Speed &rhs)
    {
        this->speedType = rhs.speedType;
        this->measurement = rhs.measurement;
    }

    bool operator == (const Speed &rhs) {
        if(this->speedType != rhs.speedType){
            return false;
        }
        if(fabs(this->measurement - rhs.measurement) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const Speed &rhs) {
        return !(*this == rhs);
    }


private:
    SPEED_TYPE speedType = SPEED_TYPE::SPEED_TYPE_ENUM_END;

    double measurement = 0.0;

};


} //end of namespace measurement
} //end of namespace mace

#endif // BASE_SPEED_H
