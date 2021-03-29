#include "state_attitude.h"

using namespace DataState;

StateAttitude::StateAttitude():
    roll(0.0),rollRate(0.0),pitch(0.0),pitchRate(0.0),yaw(0.0),yawRate(0.0)
{

}

StateAttitude::StateAttitude(const StateAttitude &attitude)
{
    this->roll = attitude.roll;
    this->rollRate = attitude.rollRate;

    this->pitch = attitude.pitch;
    this->pitchRate = attitude.pitchRate;

    this->yaw = attitude.yaw;
    this->yawRate = attitude.yawRate;
}

StateAttitude::StateAttitude(const mace_attitude_t &att):
    roll(att.roll), pitch(att.pitch), yaw(att.yaw)
{

}

StateAttitude::StateAttitude(const mace_attitude_rates_t &attRates):
    rollRate(attRates.rollspeed), pitchRate(attRates.pitchspeed), yawRate(attRates.yawspeed)
{

}

StateAttitude::StateAttitude(const mace_attitude_t &att, const mace_attitude_rates_t &attRates):
    roll(att.roll), pitch(att.pitch), yaw(att.yaw),
    rollRate(attRates.rollspeed), pitchRate(attRates.pitchspeed), yawRate(attRates.yawspeed)
{

}

StateAttitude::StateAttitude(const mace_attitude_state_full_t &att):
    roll(att.roll), pitch(att.pitch), yaw(att.yaw),
    rollRate(att.rollspeed), pitchRate(att.pitchspeed), yawRate(att.yawspeed)
{

}

void StateAttitude::setAttitude(const double &roll, const double &pitch, const double &yaw)
{
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
}

void StateAttitude::setAttitudeRates(const double &rollRate, const double &pitchRate, const double &yawRate)
{
    this->rollRate = rollRate;
    this->pitchRate =pitchRate;
    this->yawRate = yawRate;
}

mace_attitude_state_full_t StateAttitude::getMACEAttitudeStateFull() const
{
    mace_attitude_state_full_t attitude;
    attitude.roll = this->roll;
    attitude.pitch = this->pitch;
    attitude.yaw = this->yaw;
    attitude.rollspeed = this->rollRate;
    attitude.pitchspeed = this->pitchRate;
    attitude.yawspeed = this->yawRate;

    return attitude;
}

mace_attitude_t StateAttitude::getMACEEuler() const
{
    mace_attitude_t euler;
    euler.roll = this->roll;
    euler.pitch = this->pitch;
    euler.yaw = this->yaw;

    return euler;
}

mace_attitude_rates_t StateAttitude::getMACEEulerRates() const
{
    mace_attitude_rates_t eulerRates;
    eulerRates.rollspeed = this->rollRate;
    eulerRates.pitchspeed = this->pitchRate;
    eulerRates.yawspeed = this->yawRate;

    return eulerRates;
}

mace_message_t StateAttitude::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_attitude_t attitude = getMACEEuler();
    mace_msg_attitude_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}

mace_message_t StateAttitude::getMACEMsg_Rates(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_attitude_rates_t attitude = getMACEEulerRates();
    mace_msg_attitude_rates_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}

mace_message_t StateAttitude::getMACEMsg_Full(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_attitude_state_full_t attitude = getMACEAttitudeStateFull();
    mace_msg_attitude_state_full_encode_chan(systemID,compID,chan,&msg,&attitude);
    return msg;
}
