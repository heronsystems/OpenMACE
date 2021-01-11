/*!
  * @file agent_parameters.h
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
  
#ifndef AGENT_PARAMETERS_H
#define AGENT_PARAMETERS_H

#include <stdint.h>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

/*!
 * \brief The AgentParams class contains agent specific parameters, including its ID, type,
 * and capabilities.
 */
class AgentParams
{
public:
    /*!
     * \brief The AGENT_MODALITY enum defines the various types of agents supported by MACE
     */
    typedef enum class AGENT_MODALITY
    {
        UAV_MULTIROTOR,
        UAV_ROTARY,
        UAV_FIXEDWING,
        UGV_SKIDSTEER,
        Unknown
    } AGENT_MODALITY;

    static std::string getAgentTypeString(AGENT_MODALITY type);

    AgentParams(AGENT_MODALITY agentType = AGENT_MODALITY::UAV_FIXEDWING);
    
    AgentParams(const AgentParams &copy);

    AGENT_MODALITY getType() const;

    double getNominalVelocity() const;
    void setNominalVelocity(double nominalVelocity);

    double getMaxFuelVolume() const;
    void setMaxFuelVolume(double maxFuelVolume);

    double getFuelConsumptionIdle() const;
    void setFuelConsumptionIdle(double fuelConsumptionIdle);

    double getFuelConsumptionTranslational() const;
    void setFuelConsumptionTranslational(double fuelConsumptionTranslational);

    double getDefaultAltitude() const;
    void setDefaultAltitude(double defaultAltitude);

    void setAgentID(const std::string &id);
    std::string getAgentID() const;

    double getAcceptanceCriteria_Loose() const;
    double getAcceptanceCriteria_Tight() const;

    double getLoiterRadius() const;
    double getTurningRadius() const;
    double getCruisingSpeed() const;
    double getCruisingAltitude() const;
    double getTargetSamplingPeriod() const;
    double getSamplingDistance() const;
    
public:
    AgentParams& operator = (const AgentParams &rhs)
    {
        this->_type = rhs._type;
        this->_agentID = rhs._agentID;
        this->_nominalVelocity = rhs._nominalVelocity;
        this->_maxFuelVolume = rhs._maxFuelVolume;
        this->_fuelConsumptionIdle = rhs._fuelConsumptionIdle;
        this->_fuelConsumptionTranslational = rhs._fuelConsumptionTranslational;
        this->_defaultAltitude = rhs._defaultAltitude;

        _acceptanceRadius_Loose = rhs._acceptanceRadius_Loose;
        _acceptanceRadius_Tight = rhs._acceptanceRadius_Tight;
        return *this;
    }

private:
    AGENT_MODALITY _type;

    std::string _agentID;

    static void setDefaultCapababilities();

    double _nominalVelocity = 16.0;

    double _maxFuelVolume                  = 10000.0;
    double _fuelConsumptionIdle            = 0.0;
    double _fuelConsumptionTranslational   = 5.0;

    double _defaultAltitude = 50.0;
    double _acceptanceRadius_Loose = 32.0;
    double _acceptanceRadius_Tight = 40.0;
    double _loiterRadius = 20.0;
    double _turningRadius = 110.0;
    double _cruiseSpeed = 20.0;
    double _cruiseAltitude = 50.0;
    double _targetSamplingPeriod = 0.5;

};

namespace std
{
template<>
struct hash<AgentParams::AGENT_MODALITY>
{
    size_t operator() (const AgentParams::AGENT_MODALITY &type) const
    {
        return hash<int>{}(static_cast<int>(type));
    }
};
} //end of namespace std

#endif // AGENT_PARAMATERS_H
