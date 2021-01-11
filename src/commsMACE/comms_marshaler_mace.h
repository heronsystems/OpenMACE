#ifndef COMMS_MARSHALER_MACE_H
#define COMMS_MARSHALER_MACE_H

#include "common/common.h"
#include "common/watchdog.h"

#include "commsmace_global.h"

#include <unordered_map>

#include "i_link_mace.h"
#include "serial_link_mace.h"
#include "udp_link_mace.h"
#include "tcp_link_mace.h"
#include "ethernet_link_mace.h"
#include "digimesh_link.h"
#include "protocol_mavlink_mace.h"

#include "i_link_events_mace.h"

#include "comms_events_mace.h"

#include "common/optional_parameter.h"

namespace CommsMACE
{

enum class LinkTypes
{
    SERIAL,
    UDP,
    DIGIMESH,
    TCP
};

enum class Protocols
{
    MAVLINK
};


//!
//! \brief The link marshaler is to coordinate a collection of links and the protocols which those links are using.
//!
//! There should only exists one protocol object for each type.
//! But there may exists multiple links of the same type and may be using the same protocol.
//!
//! When multiple links are using the same protocol, they utilize protocol channels to differentiate themselves.
//!
class COMMSMACESHARED_EXPORT CommsMarshaler : public Publisher<CommsEvents>, private ILinkEvents, private IProtocolMavlinkEvents
{
private:

    std::unordered_map<const ILink*, std::function<void(const Resource &resource)>> m_AddedModuleAction;
    std::unordered_map<const ILink*, std::function<void(const Resource &resource)>> m_RemovedModuleAction;

public:

    //////////////////////////////////////////////////////////////
    /// Setup
    //////////////////////////////////////////////////////////////

    CommsMarshaler();

    void SpecifyAddedModuleAction(const std::string &linkName, const std::function<void(const Resource &resource)> &lambda)
    {
        m_AddedModuleAction.insert({m_CreatedLinksNameToPtr.at(linkName).get(), lambda});
    }

    void SpecifyRemovedModuleAction(const std::string &linkName, const std::function<void(const Resource &resource)> &lambda)
    {
        m_RemovedModuleAction.insert({m_CreatedLinksNameToPtr.at(linkName).get(), lambda});
    }

    //!
    //! \brief Create a mavlink protocol to be used as transport layer of a link
    //! \param config Configuration of mavlink
    //!
    void AddProtocol(const MavlinkConfiguration &config);


    //!
    //! \brief Adds a serial link that can be used
    //! \param name Name of link for use when referencing it later
    //! \param config Configuration of serial link
    //!
    void AddLink(const std::string &name, const SerialConfiguration &config);


    //!
    //! \brief Adds a UDP link that can be used
    //! \param name Name of link for use when referencing it later
    //! \param config Configuration of UDP link
    //!
    void AddUDPLink(const std::string &name, const UdpConfiguration &config);


    //!
    //! \brief Adds a TCP link that can be used
    //! \param name Name of link for use when referencing it later
    //! \param config Configuration of TCP link
    //!
    void AddTCPLink(const std::string &name, const TcpConfiguration &config);

    void AddEthernetLink(const std::string &name, const EthernetConfiguration &config);

    //!
    //! \brief Adds a DigiMesh link that can be used
    //! \param name Name of link for use when referencing later
    //! \param config Configuration of DigiMesh link
    //!
    void AddDigiMeshLink(const std::string &name, const DigiMeshConfiguration &config);


    //!
    //! \brief Add a vechile that will be communicating out of this link
    //! \param name Name of link
    //! \param vehicleID ID of vehicle
    //!
    void AddResource(const std::string &name, const Resource &resource);


    bool HasResource(const std::string &name, const Resource &resource) const;


    void RequestRemoteResources(const std::string &name, const std::vector<Resource> &expected = {});


    //!
    //! \brief Set the protocol which a link is to use
    //! \param linkName Link name to set protocol of
    //! \param protocol Protocol type that link is to use
    //!
    void SetProtocolForLink(const std::string &linkName, Protocols protocol);


    //!
    //! \brief Connect to an already created link
    //! \param linkName Name of link to connect to
    //! \return True if connection succesfull, false otherwise
    //!
    bool ConnectToLink(const std::string &linkName);


    //////////////////////////////////////////////////////////////
    /// Query
    //////////////////////////////////////////////////////////////

    //!
    //! \brief Get the channel being used by the given link to communicate
    //! \param link Link to be used
    //! \return Channel for that link
    //!
    uint8_t GetProtocolChannel(const std::string &linkName) const;


    //!
    //! \brief Issue a message to a given link
    //!
    //! The type used in the shall be an underlaying type which the protocol understands
    //! \param link Link to send message on
    //! \param message Message to send
    //!
    template <typename T>
    void SendMACEMessage(const std::string &linkName, const T& message, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());




private:

    //////////////////////////////////////////////////////////////
    /// React to Link Events
    //////////////////////////////////////////////////////////////

    virtual void AddedExternalResource(ILink *link_ptr, const Resource &resource);

    virtual void RemovedExternalResource(ILink *link_ptr, const Resource &resource) const;

    virtual void ReceiveData(ILink *link_ptr, const std::vector<uint8_t> &buffer) const;

    virtual void CommunicationError(const ILink* link_ptr, const std::string &type, const std::string &msg) const;

    virtual void CommunicationUpdate(const ILink *link_ptr, const std::string &name, const std::string &msg) const;

    virtual void Connected(const ILink* link_ptr) const;

    virtual void ConnectionRemoved(const ILink *link_ptr) const;


    //////////////////////////////////////////////////////////////
    /// Generic Protocol Events
    //////////////////////////////////////////////////////////////


    //!
    //! \brief A message about protocol has been generated
    //! \param linkName Link identifier which generated call
    //! \param title
    //! \param message
    //!
    virtual void ProtocolStatusMessage(const ILink* link_ptr, const std::string &title, const std::string &message) const;



    virtual void ReceiveLossPercentChanged(const ILink* link_ptr, int uasId, float lossPercent) const;
    virtual void ReceiveLossTotalChanged(const ILink* link_ptr, int uasId, int totalLoss) const;




    //////////////////////////////////////////////////////////////
    /// MAVLINK Protocol Events
    //////////////////////////////////////////////////////////////


    //!
    //! \brief A Message has been received over Mavlink protocol
    //! \param linkName Link identifier which generated call
    //! \param message Message that has been received
    //!
    virtual void MessageReceived(const ILink* link_ptr, const mavlink_message_t &message) const;

    //!
    //! \brief A new radio status packet received
    //! \param link
    //! \param rxerrors
    //! \param fixed
    //! \param rssi
    //! \param remrssi
    //! \param txbuf
    //! \param noise
    //! \param remnoise
    //!
    virtual void RadioStatusChanged(const ILink* link_ptr, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const;


private:

    std::unordered_map<Protocols, std::shared_ptr<IProtocol>, EnumClassHash> m_ProtocolObjects;

    std::unordered_map<std::string, std::shared_ptr<ILink>> m_CreatedLinksNameToPtr;
    std::unordered_map<const ILink*, std::string> m_CreatedLinksPtrToName;

    std::unordered_map<const ILink*, Protocols> m_LinksProtocol;

private:

    int m_MavlinkChannelsUsedBitMask;

    std::unordered_map<ILink*, uint8_t> m_MavlinkChannels;

    std::unordered_map<const ILink*, std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>> m_LastRemoteResourceRequestTime;

    std::unordered_map<const ILink*, std::vector<std::tuple<Resource, uint8_t>>> m_ExpectedResource;

    ContinuousWatchdog *m_ExpectedResourceWatchdog;

};

}//END Comms

#endif // LINKMARSHALER_H
