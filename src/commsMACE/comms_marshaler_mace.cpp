#include "comms_marshaler_mace.h"

namespace CommsMACE
{

static const int TIME_INTERVAL_BETWEEN_CONSECUTIVE_REMOTE_RESOURCE_REQUEST_IN_MS = 5000;


//////////////////////////////////////////////////////////////
/// Setup
//////////////////////////////////////////////////////////////

CommsMarshaler::CommsMarshaler() :
    m_MavlinkChannelsUsedBitMask(1),
    m_ExpectedResourceWatchdog(nullptr)
{

}


//!
//! \brief Create a mavlink protocol to be used as transport layer of a link
//! \param config Configuration of mavlink
//!
void CommsMarshaler::AddProtocol(const MavlinkConfiguration &config)
{
    if(m_ProtocolObjects.find(Protocols::MAVLINK) != m_ProtocolObjects.cend())
        throw std::runtime_error("Mavlink protocol has already been created");

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
    m_LastRemoteResourceRequestTime.insert({link.get(), {}});
    m_ExpectedResource.insert({link.get(), {}});
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
    m_LastRemoteResourceRequestTime.insert({link.get(), {}});
    m_ExpectedResource.insert({link.get(), {}});
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
    m_LastRemoteResourceRequestTime.insert({link.get(), {}});
    m_ExpectedResource.insert({link.get(), {}});
    link->AddListener(this);
}


//!
//! \brief Adds a DigiMesh link that can be used
//! \param name Name of link for use when referencing later
//! \param config Configuration of DigiMesh link
//!
void CommsMarshaler::AddDigiMeshLink(const std::string &name, const DigiMeshConfiguration &config)
{
    if(m_CreatedLinksNameToPtr.find(name) != m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<DigiMeshLink>(config);

    m_CreatedLinksNameToPtr.insert({name, link});
    m_CreatedLinksPtrToName.insert({link.get(), name});
    m_LastRemoteResourceRequestTime.insert({link.get(), {}});
    m_ExpectedResource.insert({link.get(), {}});
    link->AddListener(this);
}


void CommsMarshaler::AddEthernetLink(const std::string &name, const EthernetConfiguration &config)
{
    if(m_CreatedLinksNameToPtr.find(name) != m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name already exists");

    std::shared_ptr<ILink> link = std::make_shared<EthernetLink>(config);

    m_CreatedLinksNameToPtr.insert({name, link});
    m_CreatedLinksPtrToName.insert({link.get(), name});
    m_LastRemoteResourceRequestTime.insert({link.get(), {}});
    m_ExpectedResource.insert({link.get(), {}});
    link->AddListener(this);
}


//!
//! \brief Add a vechile that will be communicating out of this link
//! \param vehicleID ID of vechile
//!
void CommsMarshaler::AddResource(const std::string &name, const Resource &resource)
{

    if(m_CreatedLinksNameToPtr.find(name) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(name);\

    link->AddResource(resource);
}

bool CommsMarshaler::HasResource(const std::string &name, const Resource &resource) const
{
    if(m_CreatedLinksNameToPtr.find(name) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(name);\

    return link->HasResource(resource);
}

void CommsMarshaler::RequestRemoteResources(const std::string &name, const std::vector<Resource> &expected)
{
    if(m_CreatedLinksNameToPtr.find(name) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(name);
    const ILink* link_ptr = link.get();

    auto LastRemoteResourceRequestTime = m_LastRemoteResourceRequestTime.at(link_ptr);

    /// check if we have made a request recently. If so abandon this function
    if(LastRemoteResourceRequestTime.find(name) != LastRemoteResourceRequestTime.cend())
    {
        std::chrono::high_resolution_clock::time_point prevTime = LastRemoteResourceRequestTime.at(name);
        std::chrono::high_resolution_clock::time_point currTime = std::chrono::high_resolution_clock::now();

        if(std::chrono::duration_cast<std::chrono::milliseconds>(currTime - prevTime).count() < TIME_INTERVAL_BETWEEN_CONSECUTIVE_REMOTE_RESOURCE_REQUEST_IN_MS)
        {
            printf("Ignoring Remote Resource Request because one was made recently\n");
            return;
        }
    }


    /// if we know of expected resources, set a timer and make sure they have been received
    if(expected.size() > 0)
    {
        // if no expected resources then start a watchdog
        // if is already expected resources then kick the watchdog to make sure the request gets a chance to get out.
        if(m_ExpectedResource.at(link.get()).size() == 0)
        {

            m_ExpectedResourceWatchdog = new ContinuousWatchdog(std::chrono::seconds(4), [link_ptr, link, this](){

                if(m_ExpectedResource.at(link_ptr).size() == 0)
                {
                    return false;
                }

                for(auto it = m_ExpectedResource.at(link_ptr).begin() ; it != m_ExpectedResource.at(link_ptr).end() ; ++it)
                {
                    if(std::get<1>(*it) >= 4)
                    {
                        throw std::runtime_error("Expected resource was unable to be retreived after 4 attempts");
                    }
                    std::get<1>(*it) = std::get<1>(*it) + 1;
                    printf("%d\n", std::get<1>(*m_ExpectedResource.at(link_ptr).begin()));
                }


                printf("Expected resource has yet to be received, asking again\n");
                link->RequestRemoteResources();
                return true;
            });
        }
        else {
            m_ExpectedResourceWatchdog->Kick();
        }

        // Add new resource to what is expected
        for(auto it = expected.cbegin() ; it != expected.cend() ; ++it)
        {
            m_ExpectedResource.at(link.get()).push_back(std::make_tuple(*it, 0));
        }


    }

    /// make request
    link->RequestRemoteResources();


    /// update time we last made request
    if(LastRemoteResourceRequestTime.find(name) == LastRemoteResourceRequestTime.cend())
    {
        LastRemoteResourceRequestTime.insert({name, std::chrono::high_resolution_clock::now()});
    }
    else
    {
        LastRemoteResourceRequestTime[name] = std::chrono::high_resolution_clock::now();
    }
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
void CommsMarshaler::SendMACEMessage(const std::string &linkName, const T& message, const OptionalParameter<Resource> &target)
{
    if(m_CreatedLinksNameToPtr.find(linkName) == m_CreatedLinksNameToPtr.cend())
        throw std::runtime_error("The provided link name does not exists");

    std::shared_ptr<ILink> link = m_CreatedLinksNameToPtr.at(linkName);

    Protocols protocol_code = m_LinksProtocol.at(link.get());
    std::shared_ptr<IProtocol> protocol_obj = m_ProtocolObjects.at(protocol_code);

    ///////////////////
    /// Define function that sends the given message
    auto func = [protocol_code, protocol_obj, link, message, target]() {
        switch(protocol_code)
        {
        case Protocols::MAVLINK:
        {
            std::shared_ptr<MavlinkProtocol> protocol = std::static_pointer_cast<MavlinkProtocol>(protocol_obj);
            protocol->SendProtocolMessage(link.get(), message, target);
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

void CommsMarshaler::AddedExternalResource(ILink *link_ptr, const Resource &resource)
{
    if(m_ExpectedResource.find(link_ptr) != m_ExpectedResource.cend())
    {
        for(auto it = m_ExpectedResource.at(link_ptr).begin() ; it != m_ExpectedResource.at(link_ptr).end() ; )
        {
            if(std::get<0>(*it) == resource)
            {
                m_ExpectedResource[link_ptr].erase(it);
            }
            else{
                ++it;
            }
        }

        /// if no longer expecting any responses quit the watchdog and delete it.
        if(m_ExpectedResource.at(link_ptr).size() == 0)
        {
            if(m_ExpectedResourceWatchdog != nullptr)
            {
                delete m_ExpectedResourceWatchdog;
                m_ExpectedResourceWatchdog = nullptr;
            }
        }
    }

    if(m_AddedModuleAction.find(link_ptr) != m_AddedModuleAction.cend())
    {
        m_AddedModuleAction.at(link_ptr)(resource);
    }
}

void CommsMarshaler::RemovedExternalResource(ILink *link, const Resource &resource) const
{
    if(m_RemovedModuleAction.find(link) != m_RemovedModuleAction.cend())
    {
        m_RemovedModuleAction.at(link)(resource);
    }
}

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

    Emit([&](const CommsEvents *ptr){ptr->LinkCommunicationError(m_CreatedLinksPtrToName.at(link_ptr), type, msg);});
}

void CommsMarshaler::CommunicationUpdate(const ILink* link_ptr, const std::string &name, const std::string &msg) const
{
    std::cout<< "CommsMarshaler::CommunicationUpdate" << std::endl;
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    Emit([&](const CommsEvents *ptr){ptr->LinkCommunicationUpdate(m_CreatedLinksPtrToName.at(link_ptr), name, msg);});
}

void CommsMarshaler::Connected(const ILink* link_ptr) const
{

    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    std::cout<<"CommsMarshaler::Connected"<<std::endl;

    Emit([&](const CommsEvents *ptr){ptr->LinkConnected(m_CreatedLinksPtrToName.at(link_ptr));});
}

void CommsMarshaler::ConnectionRemoved(const ILink* link_ptr) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    Emit([&](const CommsEvents *ptr){ptr->LinkConnectionRemoved(m_CreatedLinksPtrToName.at(link_ptr));});
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

    Emit([&](const CommsEvents *ptr){ptr->ProtocolStatusMessage(m_CreatedLinksPtrToName.at(link_ptr), title, message);});
}


void CommsMarshaler::ReceiveLossPercentChanged(const ILink* link_ptr, int uasId, float lossPercent) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    Emit([&](const CommsEvents *ptr){ptr->ReceiveLossPercentChanged(m_CreatedLinksPtrToName.at(link_ptr), uasId, lossPercent);});
}

void CommsMarshaler::ReceiveLossTotalChanged(const ILink* link_ptr, int uasId, int totalLoss) const
{
    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    Emit([&](const CommsEvents *ptr){ptr->ReceiveLossTotalChanged(m_CreatedLinksPtrToName.at(link_ptr), uasId, totalLoss);});
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
//    std::cout<< "CommsMarshaler::MessageReceived" << std::endl;

    if(m_CreatedLinksPtrToName.find(link_ptr) == m_CreatedLinksPtrToName.cend())
        throw std::runtime_error("Provided link does not exists");

    Emit([&](CommsEvents *ptr){ptr->MACEMessage(m_CreatedLinksPtrToName.at(link_ptr), message);});
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

    Emit([&](const CommsEvents *ptr){ptr->RadioStatusChanged(m_CreatedLinksPtrToName.at(link_ptr), rxerrors, fixed, rssi, remrssi, txbuf, noise, remnoise);});
}






template void CommsMarshaler::SendMACEMessage<mavlink_message_t>(const std::string &, const mavlink_message_t&, const OptionalParameter<Resource> &target);

}//END Comms
