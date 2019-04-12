#ifndef ILINK_MACE_H
#define ILINK_MACE_H

#include <cstdlib>
#include <memory>
#include <vector>
#include <functional>
#include <QThread>

#include "i_link_events_mace.h"
#include "common/optional_parameter.h"

namespace CommsMACE
{

class ILink
{
public:

    ILink()
    {

    }

    void AddListener(ILinkEvents* ptr)
    {
        m_Listeners.push_back(ptr);
    }

    void EmitEvent(const std::function<void(ILinkEvents*)> &func) const
    {
        for(ILinkEvents* listener : m_Listeners)
        {
            func(listener);
        }
    }

    //!
    //! \brief Set the name of the link
    //! \param str Name of link
    //!
    virtual void SetLinkName(const std::string &str)
    {
        m_LinkName = str;
    }


    //!
    //! \brief Get the name of link
    //! \return Name of link
    //!
    virtual std::string GetLinkName() const
    {
        return m_LinkName;
    }

    virtual void RequestReset() = 0;

    virtual void WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>()) = 0;

    virtual void AddResource(const Resource &resource) = 0;

    virtual bool HasResource(const Resource &resource) const = 0;

    virtual void RequestRemoteResources() const = 0;


    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const = 0;


    //!
    //! \brief Get the maximum connection speed for this interface.
    //! \return The nominal data rate of the interface in bit per second, 0 if unknown
    //!
    virtual uint64_t getConnectionSpeed() const = 0;


    virtual bool Connect(void) = 0;
    virtual void Disconnect(void) = 0;

    virtual void MarshalOnThread(std::function<void()> func) = 0;

private:

    std::string m_LinkName;

    std::vector<ILinkEvents*> m_Listeners;
};

} //END Comms

#endif // ILINK_H
