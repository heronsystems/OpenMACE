#include "tcp_configuration_mace.h"

#include <iostream>

namespace CommsMACE
{

TcpConfiguration::TcpConfiguration(const std::string& tcpServerAddress, const int &tcpServerPort)
{
    _tcpServerAddress   = tcpServerAddress;
    _tcpServerPort      = tcpServerPort;
    _udpBroadcastPort   = 12345;
}

TcpConfiguration::~TcpConfiguration()
{
}

TcpConfiguration::TcpConfiguration(const std::string& tcpServerAddress, const int &tcpServerPort, const int &udpBroadcastPort)
{
    _tcpServerAddress   = tcpServerAddress;
    _tcpServerPort      = tcpServerPort;
    _udpBroadcastPort   = udpBroadcastPort;
}

TcpConfiguration::TcpConfiguration(TcpConfiguration* copy)
{
    _tcpServerAddress    = copy->tcpServerAddress();
    _tcpServerPort       = copy->tcpServerPort();
    _udpBroadcastPort    = copy->udpBroadcastPort();
}

void TcpConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    TcpConfiguration* ssource = dynamic_cast<TcpConfiguration*>(source);
    Q_ASSERT(ssource != nullptr);
    _tcpServerAddress    = ssource->tcpServerAddress();
    _tcpServerPort       = ssource->tcpServerPort();
    _udpBroadcastPort    = ssource->udpBroadcastPort();
}

void TcpConfiguration::setTCPServerAddress(std::string address)
{
    _tcpServerAddress = address;
}

void TcpConfiguration::setTCPServerPort(int portNumber)
{
    _tcpServerPort = portNumber;
}

void TcpConfiguration::setUDPBroadcastPort(int portNumber)
{
    _udpBroadcastPort = portNumber;
}


}
