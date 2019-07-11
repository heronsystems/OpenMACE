#ifndef STATE_ATTITUDE_H
#define STATE_ATTITUDE_H

#include <iostream>
#include "mace.h"

namespace DataState {

class StateAttitude
{
public:
    StateAttitude();
    StateAttitude(const StateAttitude &attitude);

    StateAttitude(const mace_attitude_t &att);
    StateAttitude(const mace_attitude_rates_t &rates);
    StateAttitude(const mace_attitude_t &att, const mace_attitude_rates_t &attRates);
    StateAttitude(const mace_attitude_state_full_t &att);

public:
    void setAttitude(const double &roll, const double &pitch, const double &yaw);
    void setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate);

public:
    mace_attitude_state_full_t getMACEAttitudeStateFull() const;
    mace_attitude_t getMACEEuler() const;
    mace_attitude_rates_t getMACEEulerRates() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;
    mace_message_t getMACEMsg_Rates(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;
    mace_message_t getMACEMsg_Full(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;


public:
    void operator = (const StateAttitude &rhs)
    {
        this->roll = rhs.roll;
        this->rollRate = rhs.rollRate;

        this->pitch = rhs.pitch;
        this->pitchRate = rhs.pitchRate;

        this->yaw = rhs.yaw;
        this->yawRate = rhs.yawRate;
    }

    bool operator == (const StateAttitude &rhs) {
        if(this->roll != rhs.roll){
            return false;
        }
        if(this->rollRate != rhs.rollRate){
            return false;
        }
        if(this->pitch != rhs.pitch){
            return false;
        }
        if(this->pitchRate != rhs.pitchRate){
            return false;
        }
        if(this->yaw != rhs.yaw){
            return false;
        }
        if(this->yawRate != rhs.yawRate){
            return false;
        }
        return true;
    }

    bool operator != (const StateAttitude &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream &operator<<(std::ostream &out, const StateAttitude &obj)
    {
        out<<"Attitude( Roll: "<<obj.roll<<", Pitch: "<<obj.pitch<<", Yaw: "<<obj.yaw<<", PitchRate: "<<obj.pitchRate<<", RollRate: "<<obj.rollRate<<", YawRate: "<<obj.yawRate<<")";
        return out;
    }

public:
    double roll;
    double pitch;
    double yaw;
    double rollRate;
    double pitchRate;
    double yawRate;

};

} //end of namespace DataState
#endif // STATE_ATTITUDE_H
