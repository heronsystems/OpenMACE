#include "command_msg_interval.h"

namespace MAVLINKUXVControllers{


void CommandMSGInterval::addIntervalRequest(const command_item::ActionMessageInterval &request)
{
    std::pair<std::map<unsigned int, command_item::ActionMessageInterval>::iterator,bool> ret;
    ret = m_MapIntervalRequest.insert ( std::pair<unsigned int, command_item::ActionMessageInterval>(request.getMessageID(),request));
    if (ret.second==false) {
        //The element already existed and therefore we should replace it
        m_MapIntervalRequest.at(request.getMessageID()) = request;
    }
}

unsigned int CommandMSGInterval::getCurrentRequestID()
{
    return m_CurrentRequestID;
}

void CommandMSGInterval::removeIntervalRequest(const unsigned int &messageID)
{
    std::map<unsigned int, command_item::ActionMessageInterval>::iterator it;
    it=m_MapIntervalRequest.find(messageID);
    if(it != m_MapIntervalRequest.end())
        m_MapIntervalRequest.erase(it);
}


bool CommandMSGInterval::removeCurrentAndTransmitNext()
{
    this->RemoveAllTransmissions(); //this should clear the existing queued object
    unsigned int currentID = getCurrentRequestID();
    removeIntervalRequest(currentID);
    return transmitNextRequest();
}

bool CommandMSGInterval::transmitNextRequest()
{
    if(m_MapIntervalRequest.size() <= 0)
        return false;

    std::map<unsigned int, command_item::ActionMessageInterval>::iterator it;
    it = m_MapIntervalRequest.begin();
    m_CurrentRequestID = it->first;
    command_item::ActionMessageInterval currentRequest = it->second;
    MavlinkEntityKey target = currentRequest.getTargetSystem();
    MavlinkEntityKey sender = currentRequest.getOriginatingSystem();
    this->Send(currentRequest,sender,target);

    return true;
}

} //end of namespace MAVLINKUXVControllers
