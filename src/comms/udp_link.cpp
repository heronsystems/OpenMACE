#include "udp_link.h"
#include <QCoreApplication>

namespace Comms
{


//!
//! \brief This class defines a thread such that a QObject can run in peace.
//!
class ReceiverThread : public QThread
{
public:
    ReceiverThread(const std::function<void(void)> &func):
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


UdpLink::UdpLink(const UdpConfiguration &config) :
    _config(config)
{
    m_socket    = nullptr;
    m_bytesRead = 0;
    m_stopp    = false;
    m_reqReset = false;

    std::cout << "Create UdpLink: " << config.listenAddress() << ":" << config.listenPortNumber() << std::endl;
}

UdpLink::~UdpLink()
{
    Disconnect();
    if(m_socket) delete m_socket;
    m_socket = nullptr;
}

void UdpLink::RequestReset()
{
    m_stoppMutex.lock();
    m_reqReset = true;
    m_stoppMutex.unlock();
}
uint64_t UdpLink::getConnectionSpeed() const
{
    // TODO: Figure out a way to have a serial ilink and udp ilink separated, as udp doesnt need connectionspeed
    return 0;
}

bool UdpLink::Connect(void)
{
    Disconnect();

    QUdpSocket::SocketError error;
    QString errorString;

    // Initialize the connection
    if (!_hardwareConnect(error, errorString)) {
        if (_config.isAutoConnect()) {
            // Be careful with spitting out open error related to trying to open a busy port using autoconnect
            if (error == QUdpSocket::AddressInUseError) {
                // Device already open, ignore and fail connect
                return false;
            }
        }

        _emitLinkError("Error connecting: Could not create port. " + errorString.toStdString());
        return false;
    }
    return true;
}

void UdpLink::Disconnect(void)
{
    if (m_socket) {

        ((ReceiverThread*)m_ListenThread)->shutdown();
        delete m_ListenThread;

        m_socket->close();

        delete m_socket;
        m_socket = nullptr;
    }
}


/// Performs the actual hardware port connection.
///     @param[out] error if failed
///     @param[out] error string if failed
/// @return success/fail
bool UdpLink::_hardwareConnect(QAbstractSocket::SocketError &error, QString& errorString)
{
    if (m_socket) {
        std::cout << "UdpLink:" << QString::number((long)this, 16).toStdString() << "closing port" << std::endl;
        m_socket->close();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        delete m_socket;
        m_socket = nullptr;
    }

    std::cout << "UdpLink: hardwareConnect to " << _config.listenAddress() << ":" << _config.listenPortNumber() << std::endl;

    m_socket = new QUdpSocket();
    m_socket->bind(QHostAddress(QString::fromStdString((_config.listenAddress()))), _config.listenPortNumber(), QUdpSocket::ShareAddress);
    //m_socket->connectToHost(QHostAddress(QString::fromStdString((_config.address()))), _config.portNumber());
    //m_socket->waitForConnected(1000);
    //m_socket->bind(_config.portNumber(), QUdpSocket::ShareAddress);

    for (int openRetries = 0; openRetries < 4; openRetries++) {
        if (!m_socket->open(QIODevice::ReadWrite)) {
            //std::cout << "Port open failed, retrying" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            break;
        }
    }

    if (!m_socket->isOpen() ) {
        //std::cerr << "open failed" << m_port->errorString().toStdString() << m_port->error() << getName() << _config.isAutoConnect() << std::endl;
        error = m_socket->error();
        errorString = m_socket->errorString();
        EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, _config.listenAddress(), "Error opening port: " + errorString.toStdString());});
        m_socket->close();
        delete m_socket;
        m_socket = nullptr;
        return false; // couldn't open udp port
    }


    // TODO: Figure out the alternative to this:
    EmitEvent([this](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, getListenAddress(), "Opened port!");});
    EmitEvent([this](const ILinkEvents *ptr){ptr->Connected(this);});

    std::cout << "Connection UdpLink: " << "with settings " << _config.listenAddress() << ":" << _config.listenPortNumber() << std::endl;


    m_ListenThread = new ReceiverThread([&](){
        if(m_socket->waitForReadyRead(2))
            this->processPendingDatagrams();

        if(m_socket->errorString() != "")
        {
//            std::cout << "Socket error: " << m_socket->errorString().toStdString() << std::endl;
        }
    });
    m_socket->moveToThread(m_ListenThread);
    m_ListenThread->start();

    return true; // successful connection
}


void UdpLink::WriteBytes(const char *bytes, int length) const
{
    QByteArray data(bytes, length);
    if(m_socket && m_socket->isOpen()) {
        // TODO: Listen for UDP messages, identify sender port, set _config.senderPort (or something), use that to send.
        //          --May want to have a senderAddress as well...
        m_socket->writeDatagram(data, QHostAddress(QString::fromStdString(_config.senderAddress())), _config.senderPortNumber());
    } else {
        // Error occured
        _emitLinkError("Could not send data - link " + getSenderAddress() + ":" + std::to_string(getSenderPortNumber()) + " is disconnected!");
    }
}



//!
//! \brief Determine the connection status
//! \return True if the connection is established, false otherwise
//!
bool UdpLink::isConnected() const
{
    bool isConnected = false;

    if (m_socket) {
        isConnected = m_socket->isOpen();
    }

    return isConnected;
}


void UdpLink::_emitLinkError(const std::string& errorMsg) const
{
    std::string msg = "Error on link " + getListenAddress() + ":" + std::to_string(getListenPortNumber()) + " - " + errorMsg;
    EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationError(this, "Link Error", msg);});
}

LinkConfiguration UdpLink::getLinkConfiguration()
{
    return _config;
}

std::string UdpLink::getListenAddress() const
{
    return _config.listenAddress();
}

int UdpLink::getListenPortNumber() const
{
    return _config.listenPortNumber();
}

std::string UdpLink::getSenderAddress() const
{
    // TODO-PAT: Handle when senderAddress has not been set, as this is an optional parameter
    return _config.senderAddress();
}

int UdpLink::getSenderPortNumber() const
{
    // TODO-PAT: Handle when senderPortNumber has not been set, as this is an optional parameter
    return _config.senderPortNumber();
}

void UdpLink::processPendingDatagrams(void)
{
    while (m_socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
        m_socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        _config.setSenderAddress(sender.toString().toStdString());
        _config.setSenderPortNumber(senderPort);

        std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(datagram.begin(), datagram.end());
        EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
    }
}

void UdpLink::linkError(QUdpSocket::SocketError error)
{
    switch (error) {
    case QUdpSocket::AddressInUseError:
        EmitEvent([this](const ILinkEvents *ptr){ptr->ConnectionRemoved(this);});
        break;
    default:
        break;
    }
}


}
