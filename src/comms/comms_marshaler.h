#ifndef COMMS_MARSHALER_H
#define COMMS_MARSHALER_H

#include "common/common.h"

#include "comms_global.h"

#include <unordered_map>

#include "i_link.h"
#include "serial_link.h"
#include "udp_link.h"
#include "tcp_link.h"
#include "protocol_mavlink.h"

#include "i_link_events.h"

#include "comms_events.h"

namespace Comms
{

enum class LinkTypes
{
    SERIAL,
    UDP
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
//! When multiple links are using the same protol, they utilize protocol channels to differiciante themselves.
//!
class COMMSSHARED_EXPORT CommsMarshaler : public Publisher<CommsEvents, std::string>, private ILinkEvents, private IProtocolMavlinkEvents
{
public:

    //////////////////////////////////////////////////////////////
    /// Setup
    //////////////////////////////////////////////////////////////

    CommsMarshaler();

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

    //!
    //! \brief DisconnectFromLink
    //! \param linkName Name of link to disconnect from
    //! \return
    //!
    bool DisconnectFromLink(const std::string &linkName);


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
    void SendMAVMessage(const std::string &linkName, const T& message);



private:

    //////////////////////////////////////////////////////////////
    /// React to Link Events
    //////////////////////////////////////////////////////////////

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
    //! \brief Heartbeat of vehicle received
    //! \param link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const ILink* link_ptr, const int &vehicleId, const mavlink_heartbeat_t &heartbeatMSG) const;


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

};

}//END Comms

#endif // LINKMARSHALER_H
