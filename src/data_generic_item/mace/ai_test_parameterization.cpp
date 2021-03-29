#include "ai_test_parameterization.h"

namespace DataGenericItem {

AI_TestParameterization::AI_TestParameterization(const AI_TestParameterization &copy)
{
    m_TestDescriptor = copy.m_TestDescriptor;
    _fieldINIFile = copy._fieldINIFile;
    m_TestOrigin = copy.m_TestOrigin;
    m_Agents = copy.m_Agents;
    _pauseAfterLoad = copy._pauseAfterLoad;
}

void AI_TestParameterization::populateMACECOMMS_TestParameterization(mavlink_ai_test_parameterization_t &obj) const
{
    obj.origin_alt = m_TestOrigin.getAltitude();
    obj.origin_lat = m_TestOrigin.getLatitude();
    obj.origin_lng = m_TestOrigin.getLongitude();

    strcpy(obj.field_file, checkStringLength(14,_fieldINIFile).c_str());
    //Ken: For now we are going to assume that there is 1 red and 1 blue to make this work of which have the same test conditions
    if(m_Agents.size() > 2)
    {
        std::cout<<"FWIW there is more than 2 agent file definitions"<<std::endl;
    }
    unsigned int currentIterator = 0;

    for (std::multimap<std::string, MLAgent_TestParameterization>::const_iterator it=m_Agents.begin(); it!=m_Agents.end(); ++it)
    {
        MLAgent_TestParameterization currentParameterization = it->second;

        if(currentIterator == 0)
        {
            obj.target_system = std::stoi(it->first);

            strcpy(obj.tc_file, checkStringLength(14, currentParameterization.getTestConditionINI()).c_str());
            strcpy(obj.file_one, checkStringLength(14, currentParameterization.getAgentINI()).c_str());
        }
        else if(currentIterator == 1)
        {
            strcpy(obj.file_two, checkStringLength(14, currentParameterization.getAgentINI()).c_str());
        }
        currentIterator++;
    }

    // Test Key:
    obj.test_id = m_TestDescriptor.getTestKey().testID;
    obj.blue_agent_type = (uint8_t)m_TestDescriptor.getTestKey().blueAgent;
    obj.red_agent_type = (uint8_t)m_TestDescriptor.getTestKey().redAgent;
}

void AI_TestParameterization::fromMACECOMMS_TestParameterization(const mavlink_ai_test_parameterization_t &obj)
{
    setFieldINI(std::string(obj.field_file));
    m_TestOrigin.updatePosition(obj.origin_lat,obj.origin_lng,obj.origin_alt);

    std::string agentID = std::to_string(obj.target_system);

    MLAgent_TestParameterization firstTestParameterization;
    firstTestParameterization.setAssociatedAgentID(agentID);
    firstTestParameterization.setINILocations(obj.file_one,obj.tc_file);

    MLAgent_TestParameterization secondTestParameterization;
    secondTestParameterization.setAssociatedAgentID(agentID);
    secondTestParameterization.setINILocations(obj.file_two,obj.tc_file);

    m_Agents.insert({std::to_string(obj.target_system), firstTestParameterization});
    m_Agents.insert({std::to_string(obj.target_system), secondTestParameterization});

    m_TestDescriptor.setTestKey(obj.test_id, obj.blue_agent_type, obj.red_agent_type);
}

std::string AI_TestParameterization::checkStringLength(const unsigned int &length, const std::string &str) const
{
    std::string rtnString = "";

    if(str.length() > length)
        rtnString = str.substr(0,(length-1));
    else
        rtnString = str;

    return rtnString;
}


} //end of namespace DataGenericItem
