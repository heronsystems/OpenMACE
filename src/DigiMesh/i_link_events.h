#ifndef I_LINK_EVENTS_H
#define I_LINK_EVENTS_H

#include <cstdlib>
#include <vector>
#include <string>

class SerialLink;

class ILinkEvents
{
public:

    virtual void ReceiveData(SerialLink *link_ptr, const std::vector<uint8_t> &buffer) = 0;

    virtual void CommunicationError(const SerialLink* link_ptr, const std::string &type, const std::string &msg) = 0;

    virtual void CommunicationUpdate(const SerialLink *link_ptr, const std::string &name, const std::string &msg) = 0;

    virtual void Connected(const SerialLink* link_ptr) = 0;

    virtual void ConnectionRemoved(const SerialLink *link_ptr) = 0;
};


#endif // I_LINK_EVENTS_H
