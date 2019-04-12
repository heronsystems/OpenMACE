#ifndef TCPCONFIGURATION_MACE_H
#define TCPCONFIGURATION_MACE_H

#include <QTcpSocket>
#include <string>

#include "link_configuration_mace.h"

#include "commsmace_global.h"

namespace CommsMACE
{

class COMMSMACESHARED_EXPORT TcpConfiguration : public LinkConfiguration
{
public:

    TcpConfiguration(const std::string &tcpServerAddress, const int &tcpServerPort);
    TcpConfiguration(const std::string &tcpServerAddress, const int &tcpServerPort, const int &udpBroadcastPort);
    TcpConfiguration(TcpConfiguration* copy);

    // Destructor
    ~TcpConfiguration();

    int tcpServerPort() const { return _tcpServerPort; }
    const std::string tcpServerAddress() const { return _tcpServerAddress; }
    int udpBroadcastPort() const { return _udpBroadcastPort; }

    void setTCPServerPort(int portNumber);
    void setTCPServerAddress(std::string address);
    void setUDPBroadcastPort(int portNumber);

    /// From LinkConfiguration
    void        copyFrom        (LinkConfiguration* source);
    //void        loadSettings    (Settings& settings, const QString& root);
    //void        saveSettings    (Settings& settings, const QString& root);
    //void        updateSettings  ();
    //QString     settingsURL     () { return "SerialSettings.qml"; }


private:
    int _tcpServerPort;
    std::string _tcpServerAddress;
    int _udpBroadcastPort;
};

}

#endif // TCPCONFIGURATION_H
