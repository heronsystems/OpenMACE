#ifndef ETHERNET_LINK_MACE_H
#define ETHERNET_LINK_MACE_H


#include "commsmace_global.h"

#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QThread>
#include <QUdpSocket>
#include <QTcpSocket>
#include <QTcpServer>

#include "ethernet_configuration.h"

#include "i_link_mace.h"


namespace CommsMACE
{

/**
 * @brief Creates an ethernet link connecting multiple computers on the same ethernet network.
 *
 * Each instance doesn't nessessarly have a unique address, they may be on the same computer.
 *
 * Resources are distributed by broadcasting over UDP.
 * When a resource is seen, a TCP socket connection is established between the two computers and the resource added.
 *
 * Later when attempting to transmit to a resource the socket pertaining to the resource is looked up and used.
 */
class COMMSMACESHARED_EXPORT EthernetLink : public ILink
{
public:

    EthernetLink(const EthernetConfiguration &config);

    ~EthernetLink();

    virtual void RequestReset();

    /**
     * @brief Send data over the link
     * @param bytes Array of data
     * @param length Length of data
     * @param target Optional target, send to all known entities if not given
     */
    virtual void WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());


    //!
    //! \brief Add a resource to the ethernet link
    //! \param resource Resource to add
    //!
    virtual void AddResource(const Resource &resource);


    //!
    //! \brief Ask if the given resource exists
    //! \param resource Resource to ask about
    //! \return True if resources exists
    //!
    virtual bool HasResource(const Resource &resource) const;


    //!
    //! \brief Issue a request to all entites for their resources
    //!
    virtual void RequestRemoteResources() const;


    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;


    uint32_t getPortNumber() const;

    //!
    //! \brief Get the maximum connection speed for this interface.
    //! \return The nominal data rate of the interface in bit per second, 0 if unknown
    //!
    virtual uint64_t getConnectionSpeed() const;

    virtual bool Connect(void);

    virtual void Disconnect(void);

    virtual void MarshalOnThread(std::function<void()> func){
        func();
    }

private:

    //!
    //! \brief Potentially read a resource sitting on m_ResourceSocket
    //!
    void ReadUDPData();


    //!
    //! \brief Process the TCP sockets.
    //!
    //! Read any pending data and write anything in the buffer
    //!
    void ProcessTCPSockets();


    //!
    //! \brief Distribute the presence of a resource over UDP Multicast
    //! \param r Resources to broadcast
    //!
    void DistributeResourcePresence(const Resource r) const;


    //!
    //! \brief Distribute the request for all resources over UDP multicast
    //!
    void DistributeResourceRequest() const;


    //!
    //! \brief Distribute broadcasted data to all parties on the UDP socket
    //! \param data Data to broadcast
    //! \param length Length
    //!
    void DistributeBroadcastedData(const char* data, const int length);


private:
    EthernetConfiguration _config;

    //! Thread to run UDP broadcasting
    QThread *m_UDPListenThread;
    QUdpSocket *m_UDPSocket;

    //! Thread to operate the TCP server to listen for new TCP connections
    QThread *m_IncommingTCPConnectionListenThread;
    QTcpServer *m_TCPConnectionServer;

    //! Thread to operate all established TCP connections
    QThread *m_TCPProcessSocketsThread;

    //! Map of local resources. i.e. resources that have been added and already distributed.
    std::vector<Resource> m_SelfResources;

    std::mutex m_WriteBufferMutex;
    std::unordered_map<std::string, std::vector<std::vector<char>>> m_WriteBuffer;

    std::unordered_map<std::string, QTcpSocket*> m_TCPSockets;
    std::unordered_map<Resource, std::string> m_RemoteResourceToAddressPortPair;
};

} //END MAVLINKComms

#endif // ETHERNET_LINK_MACE_H
