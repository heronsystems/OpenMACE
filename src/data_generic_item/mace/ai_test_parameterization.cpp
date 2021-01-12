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

} //end of namespace DataGenericItem
