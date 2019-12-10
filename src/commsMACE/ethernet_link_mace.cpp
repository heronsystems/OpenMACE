#include "ethernet_link_mace.h"

#include <QDataStream>
#include <QHostAddress>

#include "receiver_thread.h"

#ifdef CAUSE_RANDOM_FAILURES
    static const int G_FAILURETEST_CHANCE = 30;
    int G_NUM_CONSECUTIVE_FAILURES = 0;
    static const int G_MAX_CONSECUTIVE_FAILURES = 2;
#endif

namespace CommsMACE
{


const uint32_t UDP_BROADCAST_PORT = 55554;
const QHostAddress MULTICAST_GROUP(QStringLiteral("239.255.43.21"));

const uint8_t RESOURCE_DISTRIBUTE_PACKET = 0x01;
const uint8_t RESOURCE_REQUEST_PACKET = 0x02;
const uint8_t BROADCASTED_DATA = 0x03;


const uint8_t DEFAULT_WAIT_ON_PROCESSING_THREADS_IN_MS = 10;



EthernetLink::EthernetLink(const EthernetConfiguration &config) :
    _config(config),
    m_UDPSocket(nullptr)
{

    m_TCPProcessSocketsThread = new ReceiverThread([&](){
        ProcessTCPSockets();
    }, DEFAULT_WAIT_ON_PROCESSING_THREADS_IN_MS);
    m_TCPProcessSocketsThread->start();

#ifdef CAUSE_RANDOM_FAILURES
    std::srand(std::time(nullptr));
#endif
}

EthernetLink::~EthernetLink()
{
    if(m_UDPSocket != nullptr)
    {
        delete m_UDPSocket;
    }
}


void EthernetLink::RequestReset()
{
}

void EthernetLink::WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target)
{
    std::vector<char> data(bytes, bytes+length);

    m_WriteBufferMutex.lock();
    if(target.IsSet() == true) {
        if(m_RemoteResourceToAddressPortPair.find(target()) == m_RemoteResourceToAddressPortPair.cend()){
            throw std::runtime_error("Unknown resource");
        }

        if(m_WriteBuffer.find(m_RemoteResourceToAddressPortPair.at(target())) == m_WriteBuffer.cend())
        {
            m_WriteBuffer.insert({m_RemoteResourceToAddressPortPair.at(target()), {}});
        }
        m_WriteBuffer[m_RemoteResourceToAddressPortPair.at(target())].push_back(data);
    }
    else {
        DistributeBroadcastedData(bytes, length);
    }
    m_WriteBufferMutex.unlock();
}


//!
//! \brief Add a resource to the ethernet link
//! \param resource Resource to add
//!
void EthernetLink::AddResource(const Resource &resource)
{
    DistributeResourcePresence(resource);
    m_SelfResources.push_back(resource);
}


//!
//! \brief Ask if the given resource exists
//! \param resource Resource to ask about
//! \return True if resources exists
//!
bool EthernetLink::HasResource(const Resource &resource) const
{
    if(m_RemoteResourceToAddressPortPair.find(resource) != m_RemoteResourceToAddressPortPair.cend()) {
        return true;
    }

    for(auto it = this->m_SelfResources.cbegin() ; it != m_SelfResources.cend() ; ++it)
    {
        if(*it == resource) {
            return true;
        }
    }

    return false;
}

void EthernetLink::RequestRemoteResources() const
{
    DistributeResourceRequest();
}


bool EthernetLink::isConnected() const
{
    return true;
}

uint32_t EthernetLink::getPortNumber() const
{
    return _config.portNumber();
}

uint64_t EthernetLink::getConnectionSpeed() const
{
    throw std::runtime_error("Not Implimented");
}


bool EthernetLink::Connect()
{
    Disconnect();

    /// Check if the given port for TCP comminucations conflicts with port for UDP broadcasting
    if(this->_config.portNumber() == UDP_BROADCAST_PORT) {
        EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationError(this, "Link Error", "Given port for TCP server matches the UDP broadcast port");});
        return false;
    }


    /// Start a new TCP server to listen for establishing connections
    m_TCPConnectionServer = new QTcpServer();
    m_IncommingTCPConnectionListenThread = new ReceiverThread([&](){
        if(m_TCPConnectionServer->hasPendingConnections() == true) {
            QTcpSocket *connection = m_TCPConnectionServer->nextPendingConnection();
            connection->setParent(nullptr);
            std::string addressPortPair = (connection->peerAddress().toString() + ":" + connection->peerPort()).toStdString();
            connection->moveToThread(m_TCPProcessSocketsThread);
            this->m_TCPSockets.insert({addressPortPair, connection});
        }
    }, DEFAULT_WAIT_ON_PROCESSING_THREADS_IN_MS);
    m_TCPConnectionServer->listen(QHostAddress::Any, _config.portNumber());
    m_TCPConnectionServer->moveToThread(m_IncommingTCPConnectionListenThread);
    m_IncommingTCPConnectionListenThread->start();



    /// Start a new UDP socket to listen for broadcasted resources
    m_UDPSocket = new QUdpSocket();
    m_UDPSocket->bind(QHostAddress::AnyIPv4, UDP_BROADCAST_PORT, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    m_UDPSocket->joinMulticastGroup(MULTICAST_GROUP);

    m_UDPListenThread = new ReceiverThread([&](){

        if(m_UDPSocket->waitForReadyRead(300))
        {
            this->ReadUDPData();
        }

    }, DEFAULT_WAIT_ON_PROCESSING_THREADS_IN_MS);
    m_UDPSocket->moveToThread(m_UDPListenThread);
    m_UDPListenThread->start();

    return true;
}

void EthernetLink::Disconnect()
{

}

//!
//! \brief Potentially read a resource sitting on m_ResourceSocket
//!
void EthernetLink::ReadUDPData()
{

    /// read datagram
    QHostAddress address;
    QByteArray datagram;
    quint16 port_rcv;
    datagram.resize(m_UDPSocket->pendingDatagramSize());
    m_UDPSocket->readDatagram(datagram.data(), datagram.size(), &address, &port_rcv);


    /// if we don't have enough to read packet size, then datagram must of been "something else"
    if(datagram.size() < 4) {
        return;
    }


    /// peek 4 bytes ahead to read the packet size
    QDataStream stream(&datagram, QIODevice::ReadOnly);
    int packetSize;
    stream >> packetSize;


    /// If we don't have a full packet, then datagram must of been "something else"
    if(datagram.size() < 4 + packetSize) {
        return;
    }

    /// read command
    uint8_t command;
    stream >> command;

    /// Process a resource distribution packet
    if(command == RESOURCE_DISTRIBUTE_PACKET)
    {

        // read the TCP port which we are to communicate to this instance
        // Read the magnitude of resource array that was sent
        uint32_t TCPPort;
        uint32_t numResources;
        stream >> TCPPort;
        stream >> numResources;

        if(TCPPort == _config.portNumber()) {
            //printf("Ignoring Resource because received TCP server is same as self\n");
            return;
        }


        // Build Resource that was sent
        Resource r;
        for(std::size_t i = 0 ; i < numResources ; i++)
        {
            uint32_t ID;
            char* name;
            stream >> name;
            stream >> ID;
            r.Add(std::string(name), ID);
        }


        // Check if given address/port pair has an open socket.
        // If not create and connect
        const std::string addressPortStr = address.toString().toStdString() + ":" + std::to_string(TCPPort);
        if(this->m_TCPSockets.find(addressPortStr) == m_TCPSockets.cend()) {

            QTcpSocket *newSocket = new QTcpSocket();
            newSocket->connectToHost(address, TCPPort);
            if(newSocket->waitForConnected(1000) == false) {
                throw std::runtime_error("Failed establish TCP connection to" + addressPortStr);
            }
            newSocket->moveToThread(m_TCPProcessSocketsThread);
            m_TCPSockets.insert({addressPortStr, newSocket});
        }

        // Register resource
        if(m_RemoteResourceToAddressPortPair.find(r) == m_RemoteResourceToAddressPortPair.cend())
        {
            m_RemoteResourceToAddressPortPair.insert({r, addressPortStr});
            EmitEvent([this, &r](ILinkEvents *ptr){ptr->AddedExternalResource(this, r);});
        }
    }

    /// Process a resource Request packet
    else if(command == RESOURCE_REQUEST_PACKET)
    {
        // read the TCP port which we are to communicate to this instance
        // Read the magnitude of resource array that was sent
        uint32_t TCPPort;
        stream >> TCPPort;

        if(TCPPort == _config.portNumber()) {
            //printf("Ignoring Resource Request because received TCP server is same as self\n");
            return;
        }

        for(auto it = m_SelfResources.cbegin() ; it != m_SelfResources.cend() ; ++it)
        {
            DistributeResourcePresence(*it);
        }
    }
    else if(command == BROADCASTED_DATA) {


        uint32_t TCPPort;
        stream >> TCPPort;

        if(TCPPort == _config.portNumber()) {
            // printf("Ignoring Broadcasted Data because received TCP server is same as self\n");
            return;
        }

        std::vector<uint8_t> data(datagram.begin() + 9, datagram.begin() + 4 + packetSize);

        EmitEvent([this,&data](const ILinkEvents *ptr){ptr->ReceiveData(this, data);});
    }
    else
    {
        throw std::runtime_error("Unknown command");
    }
}


//!
//! \brief Process the TCP sockets.
//!
//! Read any pending data and write anything in the buffer
//!
void EthernetLink::ProcessTCPSockets()
{
    /// Read any pending data
    for(auto it = m_TCPSockets.cbegin() ; it != m_TCPSockets.cend() ; ++it)
    {
        QTcpSocket* socket = it->second;
        if(socket->bytesAvailable())
        {
            QByteArray buffer = socket->readAll();
            std::vector<uint8_t> data(buffer.begin(), buffer.end());

            EmitEvent([this,&data](const ILinkEvents *ptr){ptr->ReceiveData(this, data);});
        }
    }


    /// Write any pending data
    m_WriteBufferMutex.lock();
    for(auto it = m_WriteBuffer.begin() ; it != m_WriteBuffer.end() ; ++it)
    {

        std::string remoteAddressPortPair = it->first;
        for(auto itt = it->second.cbegin() ; itt != it->second.cend() ; ++itt)
        {
            std::vector<char> data = *itt;
            QTcpSocket* socket = this->m_TCPSockets[remoteAddressPortPair];

            socket->write(reinterpret_cast<const char*>(data.data()), data.size());
        }
        it->second.clear();
    }
    m_WriteBufferMutex.unlock();
}


//!
//! \brief Distribute the presence of a resource over UDP Multicast
//! \param r Resources to broadcast
//!
void EthernetLink::DistributeResourcePresence(const Resource r) const
{
    uint32_t TCPListenPort = this->_config.portNumber();

    std::size_t packetSize = 1+4+4;
    for(std::size_t i = 0 ; i < r.Size() ; i++)
    {
        packetSize += 4 + r.NameAt(i).length() + 4;
    }

    QByteArray datagram = "";
    QDataStream stream(&datagram, QIODevice::WriteOnly);
    stream << (quint32)packetSize;
    stream << RESOURCE_DISTRIBUTE_PACKET;
    stream << TCPListenPort;
    stream << (quint32)r.Size();
    for(std::size_t i = 0 ; i < r.Size() ; i++) {
        const char* name = r.NameAt(i).c_str();
        stream.writeBytes(name, strlen(name));
        stream << r.IDAt(i);
    }

    m_UDPSocket->writeDatagram(datagram, MULTICAST_GROUP, UDP_BROADCAST_PORT);
}

//!
//! \brief Distribute the request for all resources over UDP multicast
//!
void EthernetLink::DistributeResourceRequest() const {

    QByteArray datagram = "";
    QDataStream stream(&datagram, QIODevice::WriteOnly);
    stream << 5;
    stream << RESOURCE_REQUEST_PACKET;
    stream << this->_config.portNumber();

    m_UDPSocket->writeDatagram(datagram, QHostAddress::Broadcast, UDP_BROADCAST_PORT);
}


//!
//! \brief Distribute broadcasted data to all parties on the UDP socket
//! \param data Data to broadcast
//! \param length Length
//!
void EthernetLink::DistributeBroadcastedData(const char* data, const int length) {

    QByteArray datagram = "";
    QDataStream stream(&datagram, QIODevice::WriteOnly);
    stream << 5+length;
    stream << BROADCASTED_DATA;
    stream << this->_config.portNumber();
    datagram.append(data, length);

    m_UDPSocket->writeDatagram(datagram, QHostAddress::Broadcast, UDP_BROADCAST_PORT);
}

} // END Comms
