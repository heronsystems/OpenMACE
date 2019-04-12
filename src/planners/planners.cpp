#include "planners.h"


namespace mace{
namespace planners {

Planners::Planners(const state_space::SpaceInformationPtr &spaceInfo):
    m_spaceInfo(std::move(spaceInfo)), m_stateBegin(nullptr), m_stateEnd(nullptr), m_CB(nullptr)
{
    createLog();
}

void Planners::setPlanningSpaceInfo(const state_space::SpaceInformationPtr spaceInfo)
{
    m_spaceInfo = spaceInfo;
}


void Planners::createLog()
{
    char* MACEPath = getenv("MACE_ROOT");
    std::string rootPath(MACEPath);

    std::string logname = rootPath + "/Planners.txt";
    std::string loggerName = "Planners";
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    mLog = spdlog::basic_logger_mt(logNameArray, logname);
    mLog->set_level(spdlog::level::debug);
}


} //end of namespace planners
} //end of namespace mace
