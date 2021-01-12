#include "ai_agent_parameters.h"

namespace DataGenericItem {

AI_AgentParamters::AI_AgentParamters() :
    _agentID(""),
    _associatedForce(Data::DogfightTeam::UNKNOWN),
    _agentType(AdeptModelType::UNKNOWN)
{
}

AI_AgentParamters::AI_AgentParamters(const AI_AgentParamters &copy)
{
    _agentID = copy._agentID;
    _associatedForce = copy._associatedForce;
    _agentType = copy._agentType;
    m_initialConditions = copy.m_initialConditions;
}

// ** SETTERS **
void AI_AgentParamters::setAgentID(const std::string &ID)
{
    _agentID = ID;
}

void AI_AgentParamters::setAgentTeam(const Data::DogfightTeam &team)
{
    _associatedForce = team;
}

void AI_AgentParamters::setAgentType(const AdeptModelType &type)
{
    _agentType = type;
}

void AI_AgentParamters::setAgentFidelity(const bool &fidelity)
{
    _virtualAgent = fidelity;
}

void AI_AgentParamters::setUpdatePeriod(const unsigned int &period)
{
    _updatePeriod = period;
}


// ** GETTERS **
std::string AI_AgentParamters::getAgentID() const
{
    return _agentID;
}

Data::DogfightTeam AI_AgentParamters::getAgentTeam() const
{
    return _associatedForce;
}

AdeptModelType AI_AgentParamters::getAgentType() const
{
    return _agentType;
}

bool AI_AgentParamters::isAgentVirtual() const
{
    return _virtualAgent;
}

unsigned int AI_AgentParamters::whatIsThePeriod() const
{
    return _updatePeriod;
}

void AI_AgentParamters::populateAgentParamsFromINI(const std::string &agentFilePath)
{
    INIReader *readerAgent = new INIReader(agentFilePath);

    if (readerAgent->ParseError() < 0)
    {
        std::cout << "Can't load ini file: " + agentFilePath << std::endl;
        return;
    }

    // Agent fields:
    std::string section_agent = "agent",
                sectionField_agentID = "agent_id",
                sectionField_agentTeam = "agent_team",
                sectionField_agentType = "agent_type",
                sectionField_agentFidelity = "fidelity",
                sectionField_updatePeriod = "update_period_msec";


    // Positioning fields:
    std::string section_start = "start",
                sectionField_xPosCart = "x_pos_cart",
                sectionField_yPosCart = "y_pos_cart",
                sectionField_zPosCart = "z_pos_cart",
                sectionField_rollDeg = "roll_deg",
                sectionField_pitchDeg = "pitch_deg",
                sectionField_yawDeg = "yaw_deg",
                sectionField_speed_ms = "speed_m_s";

    std::string id, team, type, fidelity, updatePeriod, xPos, yPos, zPos, rollDeg, pitchDeg, yawDeg, speed_ms = "";
    // Set agent meta data:
    _agentID = readerAgent->Get(section_agent, sectionField_agentID, id);
    _associatedForce = Data::DogfightTeamFromString(readerAgent->Get(section_agent, sectionField_agentTeam, team));
    _agentType = AdeptModelFromString(readerAgent->Get(section_agent, sectionField_agentType, type));
    fidelity = readerAgent->Get(section_agent, sectionField_agentFidelity, fidelity);
    _updatePeriod = std::stoi(readerAgent->Get(section_agent, sectionField_updatePeriod, updatePeriod));
    if(fidelity == "virtual"){
        _virtualAgent = true;
    } else {
       _virtualAgent = false;
    }


    // Set agent start position:
    xPos = readerAgent->Get(section_start, sectionField_xPosCart, xPos);
    yPos = readerAgent->Get(section_start, sectionField_yPosCart, yPos);
    zPos = readerAgent->Get(section_start, sectionField_zPosCart, zPos);
    mace::pose::CartesianPosition_3D pos(std::stod(xPos), std::stod(yPos), std::stod(zPos));
    m_initialConditions.setPosition(pos);

    // Set agent start attitude:
    rollDeg = readerAgent->Get(section_start, sectionField_rollDeg, rollDeg);
    pitchDeg = readerAgent->Get(section_start, sectionField_pitchDeg, pitchDeg);
    yawDeg = readerAgent->Get(section_start, sectionField_yawDeg, yawDeg);
    double rollRad = std::stod(rollDeg) * M_PI/180;
    double pitchRad = std::stod(pitchDeg) * M_PI/180;
    double yawRad = std::stod(yawDeg) * M_PI/180;
    mace::pose::Rotation_3D rot(rollRad, pitchRad, yawRad);
    m_initialConditions.setRotation(rot);

    // Set agent start speed:
    speed_ms = readerAgent->Get(section_start, sectionField_speed_ms, speed_ms);
    m_initialConditions.setSpeed(std::stod(speed_ms));

    // Clean up INI reader:
    delete readerAgent;
    readerAgent = nullptr;
}

} //end of namespace DataGenericItem
