#ifndef DATA_GENERIC_ITEM_WEATHER_H
#define DATA_GENERIC_ITEM_WEATHER_H

#include <iostream>

namespace DataGenericItem {

class DataGenericItem_Weather
{
public:
    DataGenericItem_Weather();

    DataGenericItem_Weather(const DataGenericItem_Weather &copyObj);

    //!
    //! \brief setDensityAltitude Set the current density altitude
    //! \param densityAltitude
    //!
    void setDensityAltitude(const double &densityAltitude){
        this->densityAltitude = densityAltitude;
    }

    //!
    //! \brief getDensityAltitude Return the current density altitude
    //! \return densityAltitude
    //!
    double getDensityAltitude() const{
        return densityAltitude;
    }

    //!
    //! \brief setWindSpeed Set the current wind speed
    //! \param windSpeed
    //!
    void setWindSpeed(const double &windSpeed){
        this->windSpeed = windSpeed;
    }

    //!
    //! \brief getWindSpeed Return the current wind speed
    //! \return windSpeed
    //!
    double getWindSpeed() const{
        return windSpeed;
    }

    //!
    //! \brief setWindDirection Set the current wind direction
    //! \param windDirection
    //!
    void setWindDirection(const double &windDirection){
        this->windDirection = windDirection;
    }

    //!
    //! \brief getWindDirection Return the current wind direction
    //! \return windDirection
    //!
    double getWindDirection() const{
        return windDirection;
    }

public:
    void operator = (const DataGenericItem_Weather &rhs)
    {
        this->densityAltitude = rhs.densityAltitude;
        this->windSpeed = rhs.windSpeed;
        this->windDirection = rhs.windDirection;
    }

    bool operator == (const DataGenericItem_Weather &rhs) {
        if(this->densityAltitude != rhs.densityAltitude){
            return false;
        }
        if(this->windSpeed != rhs.windSpeed){
            return false;
        }
        if(this->windDirection != rhs.windDirection){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Weather &rhs) {
        return !(*this == rhs);
    }

protected:
    double  densityAltitude; // (ft)
    double  windSpeed; // (knots)
    double  windDirection; // compass heading (degrees)
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_WEATHER_H
