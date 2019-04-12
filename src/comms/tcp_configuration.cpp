#include "tcp_configuration.h"

#include <iostream>

namespace Comms
{

TcpConfiguration::TcpConfiguration(const std::string& listenAddress, const int &listenPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
//    _senderAddress       = senderAddress;
//    _senderPortNumber    = senderPort;

    //mtb - configuration only mean to hold parmaters, not tehe object itself
    //m_socket = new QUdpSocket();
}

TcpConfiguration::~TcpConfiguration()
{
    //mtb - configuration only mean to hold parmaters, not tehe object itself
    // Delete socket and break out of this loop:
    /*
    if (m_socket) {
        m_socket->close();
        delete m_socket;
        m_socket = NULL;
    }
    */
}

TcpConfiguration::TcpConfiguration(const std::string& listenAddress, const int &listenPort, const std::string& senderAddress, const int &senderPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
    _senderAddress       = senderAddress;
    _senderPortNumber    = senderPort;
}

TcpConfiguration::TcpConfiguration(TcpConfiguration* copy)
{
    _listenAddress       = copy->listenAddress();
    _listenPortNumber    = copy->listenPortNumber();
    _senderAddress       = copy->senderAddress();
    _senderPortNumber    = copy->senderPortNumber();
}

void TcpConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    TcpConfiguration* ssource = dynamic_cast<TcpConfiguration*>(source);
    Q_ASSERT(ssource != NULL);
    _listenAddress               = ssource->listenAddress();
    _listenPortNumber            = ssource->listenPortNumber();
    _senderAddress               = ssource->senderAddress();
    _senderPortNumber            = ssource->senderPortNumber();
}

void TcpConfiguration::setListenAddress(std::string address)
{
    _listenAddress = address;
}

void TcpConfiguration::setListenPortNumber(int portNumber)
{
    _listenPortNumber = portNumber;
}

void TcpConfiguration::setSenderAddress(std::string address)
{
    _senderAddress = address;
}

void TcpConfiguration::setSenderPortNumber(int portNumber)
{
    _senderPortNumber = portNumber;
}


}
