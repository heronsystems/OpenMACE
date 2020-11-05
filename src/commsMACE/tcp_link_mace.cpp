#include "tcp_link_mace.h"
#include <QCoreApplication>
#include <QHostAddress>

#include "common/common.h"

#include "tcp_link_mace.h"
#include "i_link_mace.h"
#include <QHostAddress>
#include <QCoreApplication>

namespace CommsMACE
{


//!
//! \brief This class defines a thread such that a QObject can run in peace.
//!
class CustomThread : public QThread
{
public:
    CustomThread(const std::function<void(void)> &func):
        m_func(func),
        m_Shutdown(false)
    {
        if(QCoreApplication::instance() == nullptr)
        {
            int argc = 0;
            char * argv[] = {(char *)"sharedlib.app"};
            pApp = new QCoreApplication(argc, argv);
        }
    }

    virtual void run()
    {
        while(true)
        {
            if(m_Shutdown == true)
            {
                break;
            }
            QCoreApplication::processEvents();
            m_func();
        }
    }

    virtual void shutdown()
    {
        m_Shutdown = true;
        this->wait();
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
    bool m_Shutdown;
};



TcpLink::TcpLink(const TcpConfiguration &config) :
    _config(config)
{
    m_udpSocket    = nullptr;
    tcpClient    = nullptr;
    m_bytesRead = 0;
    m_stopp    = false;
    m_reqReset = false;
}

TcpLink::~TcpLink()
{
    // Clean up threads:
    if(serverThread != nullptr)
    {
        ((CustomThread*)serverThread)->shutdown();
        delete serverThread;
    }

    for(QThread* thread : clientThreads) {
        if(thread != nullptr) {
            ((CustomThread*)thread)->shutdown();
            delete thread;
        }
    }

    // Disconnect server and clients:
    Disconnect();
}

void TcpLink::RequestReset()
{
    m_stoppMutex.lock();
    m_reqReset = true;
    m_stoppMutex.unlock();
}
uint64_t TcpLink::getConnectionSpeed() const
{
    // TODO: Figure out a way to have a serial ilink and udp ilink separated, as udp doesnt need connectionspeed
    return 0;
}

bool TcpLink::Connect(void)
{
    Disconnect();

    this->StartTCPServer();
    this->StartUDPListener();

//    m_Link = new MACEDigiMeshWrapper<MACE_INSTANCE_STR, VEHICLE_STR, GROUNDSTATION_STR, MLSTATION_STR, RTA_STR, EXTERNAL_LINK_STR>(_config.portName(), _config.baud());

//    m_Link->AddHandler_NewRemoteComponentItem_Generic([this](const ResourceKey &resourceKey, const ResourceValue &resourceValue, uint64_t addr){
//        UNUSED(addr);

//        Resource r;
//        for(std::size_t i = 0 ; i < resourceKey.size() ; i++)
//        {
//            r.Add(resourceKey.at(i), resourceValue.at(i));
//        }

//        EmitEvent([this, &r](ILinkEvents *ptr){ptr->AddedExternalResource(this, r);});
//    });

//    m_Link->AddHandler_Data([this](const std::vector<uint8_t> &data){
//        EmitEvent([this,&data](const ILinkEvents *ptr){ptr->ReceiveData(this, data);});
//    });

//    return true;

    return true;
}

void TcpLink::AddResource(const Resource &resource)
{
    //convert the target into a datastructure that MACE can understand
    ResourceKey key;
    ResourceValue value;

    for(std::size_t i = 0 ; i < resource.Size() ; i++)
    {
        key.AddNameToResourceKey(resource.NameAt(i));
        value.AddValueToResourceKey(resource.IDAt(i));
    }

    // Add resource:
//    m_Link->AddResource(key, value);
}

bool TcpLink::HasResource(const Resource &resource) const
{
    //convert the target into a datastructure that digimesh library can understand
    ResourceKey key;
    ResourceValue value;

    for(std::size_t i = 0 ; i < resource.Size() ; i++)
    {
        key.AddNameToResourceKey(resource.NameAt(i));
        value.AddValueToResourceKey(resource.IDAt(i));
    }

//    return m_Link->HasResource(key, value);
    return true;
}

void TcpLink::RequestRemoteResources() const
{
//    return m_Link->RequestRemoteResources();
    return;
}


void TcpLink::Disconnect(void)
{
    if(tcpServer) {
        tcpServer->close();
    }

    if (tcpClient) {
        tcpClient->close();
        delete tcpClient;
    }

    if (m_udpSocket) {
        if(m_UDPListenThread != nullptr) {
            ((CustomThread*)m_UDPListenThread)->shutdown();
            delete m_UDPListenThread;
        }

        m_udpSocket->close();

        delete m_udpSocket;
        m_udpSocket = nullptr;
    }
}

//!
//! \brief Starts the TCP server for other entities to send requests to
//! \return
//!
bool TcpLink::StartTCPServer()
{
    std::cout << "TcpLink::StartTCPServer" << std::endl;

    if(serverStarted)
    {
        std::cout << "Server is already listening" << std::endl;
    }
    else
    {
        std::cout << "Starting new server thread!" << std::endl;

        tcpServer = std::make_shared<QTcpServer>();
        serverThread = new CustomThread([&](){
            if(tcpServer->hasPendingConnections())
                this->on_newConnection();
        });


        tcpServer->listen(QHostAddress::Any, _config.tcpServerPort());

        tcpServer->moveToThread(serverThread);
        serverThread->start();


        if(!tcpServer->isListening())
        {
            std::cout << "Server could not start..." << std::endl;
        }
        else
        {
            std::cout << "_MACE: TCP Server started" << std::endl;
            serverStarted= true;
        }

                  // WriteBytes("hello", 5);
    }


    return tcpServer->isListening();
}

bool TcpLink::StartUDPListener() {
    if (m_udpSocket) {
        std::cout << "UdpLink:" << QString::number((long)this, 16).toStdString() << "closing port" << std::endl;
        m_udpSocket->close();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        delete m_udpSocket;
        m_udpSocket = nullptr;
    }

    m_udpSocket = new QUdpSocket();
    m_udpSocket->bind(QHostAddress::Any, _config.udpBroadcastPort(), QUdpSocket::ShareAddress);

    for (int openRetries = 0; openRetries < 4; openRetries++) {
        if (!m_udpSocket->open(QIODevice::ReadWrite)) {
            //std::cout << "Port open failed, retrying" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            break;
        }
    }

    if (!m_udpSocket->isOpen() ) {
        //std::cerr << "open failed" << m_port->errorString().toStdString() << m_port->error() << getName() << _config.isAutoConnect() << std::endl;
//        error = m_udpSocket->error();
//        errorString = m_udpSocket->errorString();
//        EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, "", "Error opening port: " + errorString.toStdString());});
        std::cout << "Error opening port: " << m_udpSocket->errorString().toStdString() << std::endl;
        m_udpSocket->close();
        delete m_udpSocket;
        m_udpSocket = nullptr;
        return false; // couldn't open udp port
    }


//    // TODO: Figure out the alternative to this:
//    EmitEvent([this](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, getListenAddress(), "Opened port!");});
//    EmitEvent([this](const ILinkEvents *ptr){ptr->Connected(this);});


    m_UDPListenThread = new CustomThread([&](){
        if(m_udpSocket->waitForReadyRead(300))
            this->processPendingDatagrams();

        if(m_udpSocket->errorString() != "")
        {
//            std::cout << "Socket error: " << m_socket->errorString().toStdString() << std::endl;
        }
    });
    m_udpSocket->moveToThread(m_UDPListenThread);
    m_UDPListenThread->start();

    std::cout << "UdpLink: listen for broadcast messages on:" << _config.udpBroadcastPort() << std::endl;

    return true; // successful connection
}

void TcpLink::on_newConnection()
{
    while (tcpServer->hasPendingConnections())
    {
        QTcpSocket *socket = tcpServer->nextPendingConnection();

        while (socket->waitForReadyRead())
        {
            QByteArray data = socket->readAll();
            std::cout << "_MACE: server reading: " << socket->bytesAvailable() << std::endl;

            QByteArray returnData("TCPLink_DataReceived");
            socket->write(returnData);
            socket->flush();
            socket->waitForBytesWritten(3000);

            std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(data.begin(), data.end());
            EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
        }

        socket->close();
    }
}

void TcpLink::processPendingDatagrams(void)
{
    while (m_udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        m_udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(datagram.begin(), datagram.end());
        EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
    }
}

void TcpLink::startTCPClient()
{
//    tcpClient = new QTcpSocket();
//    tcpClient->connectToHost(QHostAddress(QString::fromStdString((_config.listenAddress()))), _config.listenPortNumber());
//    tcpClient->waitForConnected(3000);

//    clientThread = new CustomThread([&](){});
}


void TcpLink::WriteBytesBroadcast(const char *bytes, int length, const OptionalParameter<Resource> &target) const
{
    UNUSED(target);
    std::cout<<"TcpLink::WriteBytes" << std::endl;
    QByteArray data(bytes, length);
    if(m_udpSocket && m_udpSocket->isOpen()) {
        m_udpSocket->writeDatagram(data, QHostAddress::Broadcast, _config.udpBroadcastPort());
    } else {
        // Error occured
        _emitLinkError("Could not send data - link :" + std::to_string(_config.udpBroadcastPort()) + " is disconnected!");
    }
}


void TcpLink::WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target)
{
    UNUSED(target);
    QByteArray data(bytes, length);
    std::shared_ptr<QTcpSocket> tcpSocket = std::make_shared<QTcpSocket>();

    //either broadcast or send to specific vehicle
//    if(target.IsSet() == true) {

        //convert the target into a datastructure that digimesh library can understand
//        ResourceKey key;
//        ResourceValue value;

//        for(std::size_t i = 0 ; i < target().Size() ; i++)
//        {
//            key.AddNameToResourceKey(target().NameAt(i));
//            value.AddValueToResourceKey(target().IDAt(i));
//        }

//            m_Link->SendData(data, key, value);

        // TODO: This should be pulled from the target or from the Resource map. For now, I'm assuming both MACE instances are on the same IP, so senderAddress == targetAddress
        // Also, this may need to be moved to a thread
        // Also also, may want to move the write into the m_Link->SendData() method like we do with the DigimeshWrapper, but we may not need to do everything the digiwrapper does...
        tcpSocket->connectToHost(QString::fromStdString(_config.tcpServerAddress()), 14551); // Hard-coded for now since I know we're on the same machine and know the port
    //    CustomThread* clientThread = new CustomThread([&](){
            tcpSocket->waitForConnected();
            if(tcpSocket->state() == QAbstractSocket::ConnectedState)
            {
                tcpSocket->write(data); //write the data itself
                tcpSocket->flush();
                tcpSocket->waitForBytesWritten();
    //            return true;
            }
            else
            {
                std::cout << "TCP socket not connected tcp_link_mace" << std::endl;
                tcpSocket->close();
    //            return false;
            }
    //    });
//    }
//    else {
////            m_Link->BroadcastData(data);
//        WriteBytesBroadcast(bytes, length);
//    }
}




//!
//! \brief Determine the connection status
//! \return True if the connection is established, false otherwise
//!
bool TcpLink::isConnected() const
{
    return tcpServer->isListening();
}


void TcpLink::_emitLinkError(const std::string& errorMsg) const
{
    std::string msg = "Error on link " + getTCPServerAddress() + ":" + std::to_string(getTCPServerPort()) + " - " + errorMsg;
    EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationError(this, "Link Error", msg);});
}

LinkConfiguration TcpLink::getLinkConfiguration()
{
    return _config;
}

std::string TcpLink::getTCPServerAddress() const
{
    return _config.tcpServerAddress();
}

int TcpLink::getTCPServerPort() const
{
    return _config.tcpServerPort();
}

int TcpLink::getUDPBroadcastPort() const
{
    // TODO-PAT: Handle when senderPortNumber has not been set, as this is an optional parameter
    return _config.udpBroadcastPort();
}

void TcpLink::processData(void)
{
        std::cout << "tcp_link: processData()" << std::endl;

        qDebug() << "Client: Connected!";





        QByteArray dataBuff(tcpClient->readAll());

        qDebug() << qPrintable(dataBuff);






        std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(dataBuff.begin(), dataBuff.end());
        EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
}

void TcpLink::linkError(QTcpSocket::SocketError error)
{
    switch (error) {
    case QTcpSocket::AddressInUseError:
        EmitEvent([this](const ILinkEvents *ptr){ptr->ConnectionRemoved(this);});
        break;
    default:
        break;
    }
}


}

