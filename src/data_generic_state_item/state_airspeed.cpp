#include "state_airspeed.h"

namespace DataState {

StateAirspeed::StateAirspeed():
    airspeed(0.0)
{

}

StateAirspeed::StateAirspeed(const StateAirspeed &copyObj)
{
    this->operator =(copyObj);
}

} //end of namespace DataState
