#include "comms_marshaler.h"

namespace Comms
{


//////////////////////////////////////////////////////////////
/// Setup
//////////////////////////////////////////////////////////////

CommsMarshaler::CommsMarshaler() :
    m_MavlinkChannelsUsedBitMask(1)
{

}


//!
//! \brief Create a mavlink protocol to be used as transport layer of a link
//! \param config Configuration of mavlink
//!
void CommsMarshaler::AddProtocol(const MavlinkConfiguration &config)
{
    if(m_ProtocolObjects.find(Protocols::MAVLINK) != m_ProtocolObjects.cend())
        return;

    std::shared_ptr<MavlinkProtocol> protocol = std::make_shared<MavlinkProtocol>(config);
    protocol->AddListner(this);

    m_ProtocolObjects.insert({Protocols::MAVLINK, protocol});
}


//!
//! \brief Adds a serial link that can be used
//! \param name Name of link for use when referencing it later
//! \param config Configuration of serial link
//!
void CommsMarshaler::AddLink(const std::string &name, const SerialConfiguration &config)
{
    if(m_CreatedLinksNameToPtr.find(name) != m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<SerialLink>(config);

    m_CreatedLinksNameToPtr.insert({name, link});
    m_CreatedLinksPtrToName.insert({link.get(), name});
    link->AddListener(this);
}


//!
//! \brief Adds a UDP link that can be used
//! \param name Name of link for use when referencing it later
//! \param config Configuration of UDP link
//!
void CommsMarshaler::AddUDPLink(const std::string &name, const UdpConfiguration &config)
{
    if(m_CreatedLinksNameToPtr.find(name) != m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<UdpLink>(config);

    m_CreatedLinksNameToPtr.insert({name, link});
    m_CreatedLinksPtrToName.insert({link.get(), name});
    link->AddListener(this);
}

//!
//! \brief Adds a TCP link that can be used
//! \param name Name of link for use when referencing it later
//! \param config Configuration of TCP link
//!
void CommsMarshaler::AddTCPLink(const std::string &name, const TcpConfiguration &config)
{
    if(m_CreatedLinksNameToPtr.find(name) != m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<TcpLink>(config);

    m_CreatedLinksNameToPtr.insert({name, link});
    m_CreatedLinksPtrToName.insert({link.get(), name});
    link->AddListener(this);
}




//!
//! \brief Set the protocol which a link is to use
//! \param linkName Link name to set protocol of
//! \param protocol Protocol type that link is to use
//!
void CommsMarshaler::SetProtocolForLink(const std::string &linkName, Protocols protocol)
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);

    ILink* link_ptr = link.get();
    if(m_LinksProtocol.find(link_ptr) == m_LinksProtocol.cend())
        m_LinksProtocol.insert({link_ptr, protocol});
    else
        m_LinksProtocol[link_ptr] = protocol;

    std::shared_ptr<IProtocol> protocolObj = m_ProtocolObjects[protocol];
    protocolObj->ResetMetadataForLink(link_ptr);


    // if mavlink then set the channel
    if(protocol == Protocols::MAVLINK)
    {

        std::shared_ptr<MavlinkProtocol> mavlinkProtocol = std::static_pointer_cast<MavlinkProtocol>(protocolObj);
        bool channelSet = false;
        for (int i=0; i<32; i++) {
            if (!(m_MavlinkChannelsUsedBitMask & 1 << i)) {
                mavlink_reset_channel_status(i);
                protocolObj->SetChannel(link_ptr, i);
                m_MavlinkChannelsUsedBitMask |= 1 << i;
                channelSet = true;
                break;
            }
        }

        if (!channelSet) {
            throw std::runtime_error("Ran out of MAVLINK channels");
        }
    }

}


//!
//! \brief Connect to an already created link
//! \param linkName Name of link to connect to
//! \return True if connection succesfull, false otherwise
//!
bool CommsMarshaler::ConnectToLink(const std::string &linkName)
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);

    return link->Connect();
}


bool CommsMarshaler::DisconnectFromLink(const std::string &linkName)
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);
    link->Disconnect();

    m_CreatedLinksNameToPtr.erase(linkName);

    return true;
}






//////////////////////////////////////////////////////////////
/// Query
//////////////////////////////////////////////////////////////


//!
//! \brief Get the channel being used by the given link to communicate
//! \param link Link to be used
//! \return Channel for that link
//!
uint8_t CommsMarshaler::GetProtocolChannel(const std::string &linkName) const
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);

    Protocols protocol = m_LinksProtocol.at(link.get());
    std::shared_ptr<IProtocol> protocolObj =  m_ProtocolObjects.at(protocol);

    return protocolObj->GetChannel(link.get());
}





//!
//! \brief Issue a message to a given link
//!
//! The type used in the shall be an underlaying type which the protocol understands
//! \param link Link to send message on
//! \param message Message to send
//!
template <typename T>
void CommsMarshaler::SendMAVMessage(const std::string &linkName, const T& message)
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);

    Protocols protocol_code = m_LinksProtocol.at(link.get());
    std::shared_ptr<IProtocol> protocol_obj = m_ProtocolObjects.at(protocol_code);

    ///////////////////
    /// Define function that sends the given message
    auto func = [protocol_code, protocol_obj, link, message]() {
        switch(protocol_code)
        {
        case Protocols::MAVLINK:
        {
            std::shared_ptr<MavlinkProtocol> protocol = std::static_pointer_cast<MavlinkProtocol>(protocol_obj);
            protocol->SendProtocolMessage(link.get(), message);
            break;
        }
        default:
            throw std::runtime_error("Attempting to send a message on an unknown protocol");
        }
    };

    link->MarshalOnThread(func);
}





//////////////////////////////////////////////////////////////
/// React to Link Events
//////////////////////////////////////////////////////////////

void CommsMarshaler::ReceiveData(ILink* link, const std::vector<uint8_t> &buffer) const
{
    if(m_LinksProtocol.find(link) == m_LinksProtocol.cend())
        throw std::runtime_error("Protocol is not set for given link");
    Protocols protocol = m_LinksProtocol.at(link);

    if(m_ProtocolObjects.find(protocol) == m_ProtocolObjects.cend())
        throw std::runtime_error("Object has not be set for link's protocol");
    std::shared_ptr<IProtocol> protocolObj = m_ProtocolObjects.at(protocol);

    protocolObj->ReceiveData(link, buffer);
}


void CommsMarshaler::CommunicationError(const ILink* link_ptr, const std::string &type, const std::string &msg) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->LinkCommunicationError(linkName, type, msg);}, linkName);
}

void CommsMarshaler::CommunicationUpdate(const ILink* link_ptr, const std::string &name, const std::string &msg) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->LinkCommunicationUpdate(linkName, name, msg);}, linkName);
}

void CommsMarshaler::Connected(const ILink* link_ptr) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->LinkConnected(linkName);}, linkName);
}

void CommsMarshaler::ConnectionRemoved(const ILink* link_ptr) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->LinkConnectionRemoved(linkName);}, linkName);
}



//////////////////////////////////////////////////////////////
/// Generic Protocol Events
//////////////////////////////////////////////////////////////


//!
//! \brief A message about protocol has been generated
//! \param linkName Link identifier which generated call
//! \param title
//! \param message
//!
void CommsMarshaler::ProtocolStatusMessage(const ILink* link_ptr, const std::string &title, const std::string &message) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->ProtocolStatusMessage(linkName, title, message);}, linkName);
}


void CommsMarshaler::ReceiveLossPercentChanged(const ILink* link_ptr, int uasId, float lossPercent) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->ReceiveLossPercentChanged(linkName, uasId, lossPercent);}, linkName);
}

void CommsMarshaler::ReceiveLossTotalChanged(const ILink* link_ptr, int uasId, int totalLoss) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->ReceiveLossTotalChanged(linkName, uasId, totalLoss);}, linkName);
}




//////////////////////////////////////////////////////////////
/// MAVLINK Protocol Events
//////////////////////////////////////////////////////////////


//!
//! \brief A Message has been received over Mavlink protocol
//! \param linkName Link identifier which generated call
//! \param message Message that has been received
//!
void CommsMarshaler::MessageReceived(const ILink* link_ptr, const mavlink_message_t &message) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](CommsEvents *ptr){ptr->MavlinkMessage(linkName, message);}, linkName);
}


//!
//! \brief Heartbeat of vehicle received
//! \param link
//! \param vehicleId
//! \param vehicleMavlinkVersion
//! \param vehicleFirmwareType
//! \param vehicleType
//!
void CommsMarshaler::VehicleHeartbeatInfo(const ILink* link_ptr, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](CommsEvents *ptr){ptr->VehicleHeartbeatInfo(linkName, systemID, heartbeatMSG);}, linkName);
}

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
void CommsMarshaler::RadioStatusChanged(const ILink* link_ptr, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::string linkName = m_CreatedLinksPtrToName.at(link_ptr);
    Emit([&](const CommsEvents *ptr){ptr->RadioStatusChanged(linkName, rxerrors, fixed, rssi, remrssi, txbuf, noise, remnoise);}, linkName);
}




template void CommsMarshaler::SendMAVMessage<mavlink_message_t>(const std::string &, const mavlink_message_t&);

}//END Comms
