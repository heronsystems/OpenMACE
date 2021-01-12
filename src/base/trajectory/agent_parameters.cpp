/*!
  * @file agent_parameters.cpp
  *
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *
  * @section DESCRIPTION
  *
  * @date
  *     Feb 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#include "agent_parameters.h"


/*!
 * \brief Constructor
 * \param id Agent ID
 * \param agentType Agent type
 */
std::string AgentParams::getAgentTypeString(AgentParams::AGENT_MODALITY type)
{
    switch (type)
    {
    case AGENT_MODALITY::UAV_MULTIROTOR:
        return "UAV_MULTIROTOR";
    case AGENT_MODALITY::UAV_FIXEDWING:
        return "UAV_FIXEDWING";
    case AGENT_MODALITY::UAV_ROTARY:
        return "UAV_ROTARY";
    case AGENT_MODALITY::UGV_SKIDSTEER:
        return "UGV_SKIDSTEER";
    case AGENT_MODALITY::Unknown:
        return "Unknown";
    }
}

AgentParams::AgentParams(AGENT_MODALITY agentType) : _type(agentType)
{

}

AgentParams::AgentParams(const AgentParams &copy)
{
    _type = copy._type;
    _agentID = copy._agentID;
    _nominalVelocity = copy._nominalVelocity;
    _maxFuelVolume = copy._maxFuelVolume;
    _fuelConsumptionIdle = copy._fuelConsumptionIdle;
    _fuelConsumptionTranslational = copy._fuelConsumptionTranslational;
    _defaultAltitude = copy._defaultAltitude;

    _acceptanceRadius_Loose = copy._acceptanceRadius_Loose;
    _acceptanceRadius_Tight = copy._acceptanceRadius_Tight;
}

/*!
 * \brief Retrieves the agent type
 * \return Agent type
 */
AgentParams::AGENT_MODALITY AgentParams::getType() const
{
    return _type;
}


void AgentParams::setAgentID(const std::string &id)
{
    _agentID = id;
}
std::string AgentParams::getAgentID() const
{
    return _agentID;
}

double AgentParams::getDefaultAltitude() const
{
    return _defaultAltitude;
}

void AgentParams::setDefaultAltitude(double defaultAltitude)
{
    _defaultAltitude = defaultAltitude;
}

double AgentParams::getFuelConsumptionTranslational() const
{
    return _fuelConsumptionTranslational;
}

void AgentParams::setFuelConsumptionTranslational(double fuelConsumptionTranslational)
{
    _fuelConsumptionTranslational = fuelConsumptionTranslational;
}

double AgentParams::getFuelConsumptionIdle() const
{
    return _fuelConsumptionIdle;
}

void AgentParams::setFuelConsumptionIdle(double fuelConsumptionIdle)
{
    _fuelConsumptionIdle = fuelConsumptionIdle;
}

double AgentParams::getMaxFuelVolume() const
{
    return _maxFuelVolume;
}

void AgentParams::setMaxFuelVolume(double maxFuelVolume)
{
    _maxFuelVolume = maxFuelVolume;
}

double AgentParams::getNominalVelocity() const
{
    return _nominalVelocity;
}

void AgentParams::setNominalVelocity(double nominalVelocity)
{
    _nominalVelocity = nominalVelocity;
}

double AgentParams::getAcceptanceCriteria_Loose() const
{
    return _acceptanceRadius_Loose;
}

double AgentParams::getAcceptanceCriteria_Tight() const
{
    return _acceptanceRadius_Tight;
}

double AgentParams::getLoiterRadius() const
{
    return _loiterRadius;
}

double AgentParams::getTurningRadius() const
{
    return _turningRadius;
}

double AgentParams::getCruisingSpeed() const
{
    return _cruiseSpeed;
}

double AgentParams::getCruisingAltitude() const
{
    return _cruiseAltitude;
}

double AgentParams::getTargetSamplingPeriod() const
{
    return _targetSamplingPeriod;
}

double AgentParams::getSamplingDistance() const
{
    return getTargetSamplingPeriod() * getCruisingSpeed();
}
