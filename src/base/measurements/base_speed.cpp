#include "base_speed.h"

namespace mace {
namespace measurements {

Speed::Speed()
{

}

Speed::Speed(const Speed &copy)
{
    this->speedType = copy.speedType;
    this->measurement = copy.measurement;
}

void Speed::updateSpeedMeasurement(const double &value)
{
    this->measurement = value;
}

void Speed::updateSpeedType(const SPEED_TYPE &type)
{
    this->speedType = type;
}

double Speed::getSpeedMeasurement() const
{
    return this->measurement;
}

SPEED_TYPE Speed::getSpeedType() const
{
    return this->speedType;
}

void Speed::setSpeed(const double &speed)
{
    this->measurement = speed;
}

double Speed::getSpeed() const
{
    return this->measurement;
}


} //end of namespace measurement
} //end of namespace mace
