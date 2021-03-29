#ifndef STATE_AIRSPEED_H
#define STATE_AIRSPEED_H

namespace DataState {

class StateAirspeed
{
public:
    StateAirspeed();
    StateAirspeed(const StateAirspeed &copyObj);

public:
    void setAirspeed(const double &speed)
    {
        this->airspeed = speed;
    }

public:
    void operator = (const StateAirspeed &rhs)
    {
        this->airspeed = rhs.airspeed;
    }

    bool operator == (const StateAirspeed &rhs) {
        if(this->airspeed != rhs.airspeed){
            return false;
        }
        return true;
    }

    bool operator != (const StateAirspeed &rhs) {
        return !(*this == rhs);
    }

public:
    double airspeed;
};

} //end of namespace DataState

#endif // STATE_AIRSPEED_H
