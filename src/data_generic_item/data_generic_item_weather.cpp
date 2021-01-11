#include "data_generic_item_weather.h"

namespace DataGenericItem{
DataGenericItem_Weather::DataGenericItem_Weather() :
    densityAltitude(0), windSpeed(0),windDirection(0)
{
}

DataGenericItem_Weather::DataGenericItem_Weather(const DataGenericItem_Weather &copyObj){
    this->densityAltitude = copyObj.getDensityAltitude();
    this->windSpeed = copyObj.getWindSpeed();
    this->windDirection = copyObj.getWindDirection();
}


} //end of namespace DataGenericItem

