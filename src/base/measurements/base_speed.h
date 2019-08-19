#ifndef BASE_SPEED_H
#define BASE_SPEED_H

#include <string>
#include <limits>
#include <cmath>

namespace mace {
namespace measurements {

class Speed
{
public:
    enum class SpeedTypes: uint8_t
    {
        AIRSPEED = 0,
        GROUNDSPEED = 1,
        UNKNOWN = 2
    };

public:
    Speed();

    Speed(const Speed &copy);

    void updateSpeedMeasurement(const double &value);

    void updateSpeedType(const SpeedTypes &type);

    double getSpeedMeasurement() const;

    SpeedTypes getSpeedType() const;


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
    SpeedTypes speedType = SpeedTypes::UNKNOWN;

    double measurement = 0.0;

};


} //end of namespace measurement
} //end of namespace mace

#endif // BASE_SPEED_H
