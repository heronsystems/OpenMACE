#include "udp_configuration_mace.h"

#include <iostream>

namespace CommsMACE
{

UdpConfiguration::UdpConfiguration(const std::string& listenAddress, const int &listenPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
//    _senderAddress       = senderAddress;
//    _senderPortNumber    = senderPort;
    m_socket = new QUdpSocket();
}

UdpConfiguration::~UdpConfiguration()
{
    // Delete socket and break out of this loop:
    if (m_socket) {
        m_socket->close();
        delete m_socket;
        m_socket = NULL;
    }
}

UdpConfiguration::UdpConfiguration(const std::string& listenAddress, const int &listenPort, const std::string& senderAddress, const int &senderPort)
{
    _listenAddress       = listenAddress;
    _listenPortNumber    = listenPort;
    _senderAddress       = senderAddress;
    _senderPortNumber    = senderPort;
}

UdpConfiguration::UdpConfiguration(UdpConfiguration* copy)
{
    _listenAddress       = copy->listenAddress();
    _listenPortNumber    = copy->listenPortNumber();
    _senderAddress       = copy->senderAddress();
    _senderPortNumber    = copy->senderPortNumber();
}

void UdpConfiguration::copyFrom(LinkConfiguration* source)
{
    LinkConfiguration::copyFrom(source);
    UdpConfiguration* ssource = dynamic_cast<UdpConfiguration*>(source);
    Q_ASSERT(ssource != NULL);
    _listenAddress               = ssource->listenAddress();
    _listenPortNumber            = ssource->listenPortNumber();
    _senderAddress               = ssource->senderAddress();
    _senderPortNumber            = ssource->senderPortNumber();
}

void UdpConfiguration::setListenAddress(std::string address)
{
    _listenAddress = address;
}

void UdpConfiguration::setListenPortNumber(int portNumber)
{
    _listenPortNumber = portNumber;
}

void UdpConfiguration::setSenderAddress(std::string address)
{
    _senderAddress = address;
}

void UdpConfiguration::setSenderPortNumber(int portNumber)
{
    _senderPortNumber = portNumber;
}



// Set up a UDP socket to listen for UDP messages on, and set the sender address and port
void UdpConfiguration::listenForPort(const int &listenPortNumber)
{
    if (m_socket) {
        m_socket->close();
    }

    m_socket->bind(QHostAddress::AnyIPv4, listenPortNumber, QUdpSocket::ShareAddress);

    if(m_socket->waitForReadyRead(300))
    {
        while (m_socket->hasPendingDatagrams())
        {
            QByteArray datagram;
            datagram.resize(m_socket->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort;
            m_socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            setSenderAddress(sender.toString().toStdString());
            setSenderPortNumber(senderPort);

            // Delete socket and break out of this loop:
            if (m_socket) {
                m_socket->close();
//                m_socket->waitForDisconnected();
                delete m_socket;
                m_socket = NULL;
            }

            break;
        }
    }
    else
    {
        this->listenForPort(listenPortNumber);
    }

}


}
