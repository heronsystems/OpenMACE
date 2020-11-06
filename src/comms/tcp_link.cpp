#include "tcp_link.h"
#include <QHostAddress>
#include <QCoreApplication>

namespace Comms
{



class ServerThread : public QThread
{
public:
    ServerThread(const std::function<void(void)> &func):
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

class ClientThread : public QThread
{
public:
    ClientThread(const std::function<void(void)> &func):
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
    tcpClient    = nullptr;
    m_bytesRead = 0;
    m_stopp    = false;
    m_reqReset = false;

    std::cout << "Create TcpLink: " << config.listenAddress() << ":" << config.listenPortNumber() << std::endl;

   // this->StartTCPServer();


}

TcpLink::~TcpLink()
{
    Disconnect();
    if(tcpClient) delete tcpClient;
    tcpClient = nullptr;
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

//    QTcpSocket::SocketError error;
//    QString errorString;

    this->StartTCPServer();
    /*
    // Initialize the connection
    if (!_hardwareConnect(error, errorString)) {
        if (_config.isAutoConnect()) {
            // Be careful with spitting out open error related to trying to open a busy port using autoconnect
            if (error == QTcpSocket::AddressInUseError) {
                // Device already open, ignore and fail connect
                return false;
            }
        }

        _emitLinkError("Error connecting: Could not create port. " + errorString.toStdString());
        return false;
    }
    */
    return true;
}

void TcpLink::Disconnect(void)
{
    if (tcpClient) {

        ((ClientThread*)clientThread)->shutdown();
        delete clientThread;

        tcpClient->close();

        delete tcpClient;
        tcpClient = nullptr;
    }
}

//!
//! \brief Starts the TCP server for other entities to send requests to
//! \return
//!
bool TcpLink::StartTCPServer()
{
        tcpServer = new QTcpServer();
        serverThread = new ServerThread([&](){
                while (tcpServer->hasPendingConnections())
                {
                    QTcpSocket *socket = tcpServer->nextPendingConnection();

                    while (socket->waitForReadyRead(1000))
                    {
                       // QByteArray dataBuff(socket->readAll());

                        qDebug() << "server reading:" << socket->bytesAvailable();
                        qDebug() << socket->readAll();



                        socket->write("hello client \r\n");
                        socket->flush();
                        socket->waitForBytesWritten(3000);
                    }

                    // TODO-PAT: Try to leave this socket open if possible??
                    socket->close();
                }
          });


        tcpServer->listen(QHostAddress::Any, _config.listenPortNumber() );

        tcpServer->moveToThread(serverThread);
        serverThread->start();


        if(!tcpServer->isListening())
        {
            std::cout << "Server could not start..." << std::endl;
        }
        else
        {
            std::cout << "TCP Server started" << std::endl;
            // TODO: Figure out the alternative to this:
            EmitEvent([this](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, getListenAddress(), "Opened port!");});
            EmitEvent([this](const ILinkEvents *ptr){ptr->Connected(this);});
        }

        return tcpServer->isListening();
    }

/// Performs the actual hardware port connection.
///     @param[out] error if failed
///     @param[out] error string if failed
/// @return success/fail
bool TcpLink::_hardwareConnect(QAbstractSocket::SocketError &error, QString& errorString)
{
    UNUSED(error);
    UNUSED(errorString);
    /*
    if (tcpClient) {
        std::cout << "TcpLink:" << QString::number((long)this, 16).toStdString() << "closing port" << std::endl;
        tcpClient->close();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        delete tcpClient;
        tcpClient = nullptr;
    }

    std::cout << "TcpLink: hardwareConnect to " << _config.listenAddress() << ":" << _config.listenPortNumber() << std::endl;

    tcpClient = new QTcpSocket();
    tcpClient->connectToHost(QHostAddress(QString::fromStdString((_config.listenAddress()))), _config.listenPortNumber());
    tcpClient->waitForConnected(3000);


    for (int openRetries = 0; openRetries < 4; openRetries++) {
        if (!tcpClient->open(QIODevice::ReadWrite)) {
            std::cout << "Port open failed, retrying" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            break;
        }
    }

    if (!tcpClient->isOpen() ) {
        std::cerr << "open failed" << std::endl;
        error = tcpClient->error();
        errorString = tcpClient->errorString();
        EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, _config.listenAddress(), "Error opening port: " + errorString.toStdString());});
        tcpClient->close();
        delete tcpClient;
        tcpClient = nullptr;
        return false; // couldn't open tcp port
    }


    // TODO: Figure out the alternative to this:
    EmitEvent([this](const ILinkEvents *ptr){ptr->CommunicationUpdate(this, getListenAddress(), "Opened port!");});
    EmitEvent([this](const ILinkEvents *ptr){ptr->Connected(this);});

    std::cout << "Connection TCPLink: " << "with settings " << _config.listenAddress() << ":" << _config.listenPortNumber() << std::endl;

   clientThread = new ClientThread([&](){

       if(tcpClient->state() == QTcpSocket::ConnectedState)
       {
         tcpClient->write("hello server");
         tcpClient->waitForBytesWritten(1000);


        while(tcpClient->waitForReadyRead(3000))
            this->processData();

        if(tcpClient->errorString() != "")
        {
            //std::cout << "Socket error: " << tcpClient->errorString().toStdString() << std::endl;
        }
       }
    });

    tcpClient->moveToThread(clientThread);
    clientThread->start();
*/
    return true;
}


void TcpLink::startTCPClient()
{
    tcpClient = new QTcpSocket();
    tcpClient->connectToHost(QHostAddress(QString::fromStdString((_config.listenAddress()))), _config.listenPortNumber());
    tcpClient->waitForConnected(3000);

    clientThread = new ClientThread([&](){});

}


void TcpLink::WriteBytes(const char *bytes, int length) const
{
    QByteArray data(bytes, length);
    std::cout << "Connection TCPLink: " << "with settings " << _config.listenAddress() << ":" << _config.listenPortNumber() << std::endl;


     if(tcpClient && tcpClient->isOpen())
     {
               tcpClient->write(data,length);
           } else {
               // Error occured
               _emitLinkError("Could not send data - link " + getSenderAddress() + ":" + std::to_string(getSenderPortNumber()) + " is disconnected!");
           }
        if(tcpClient->errorString() != "")
        {
            //std::cout << "Socket error: " << tcpClient->errorString().toStdString() << std::endl;
        }


    tcpClient->moveToThread(clientThread);
    clientThread->start();

}



//!
//! \brief Determine the connection status
//! \return True if the connection is established, false otherwise
//!
bool TcpLink::isConnected() const
{
    bool isConnected = false;

    if (tcpClient) {
        isConnected = tcpClient->isOpen();
    }

    return isConnected;
}


void TcpLink::_emitLinkError(const std::string& errorMsg) const
{
    std::string msg = "Error on link " + getListenAddress() + ":" + std::to_string(getListenPortNumber()) + " - " + errorMsg;
    EmitEvent([&](const ILinkEvents *ptr){ptr->CommunicationError(this, "Link Error", msg);});
}

LinkConfiguration TcpLink::getLinkConfiguration()
{
    return _config;
}

std::string TcpLink::getListenAddress() const
{
    return _config.listenAddress();
}

int TcpLink::getListenPortNumber() const
{
    return _config.listenPortNumber();
}

std::string TcpLink::getSenderAddress() const
{
    // TODO-PAT: Handle when senderAddress has not been set, as this is an optional parameter
    return _config.senderAddress();
}

int TcpLink::getSenderPortNumber() const
{
    // TODO-PAT: Handle when senderPortNumber has not been set, as this is an optional parameter
    return _config.senderPortNumber();
}

void TcpLink::processData(void)
{
        std::cout << "tcp_link: processData()" << std::endl;

        qDebug() << "Client: Connected!";





        QByteArray dataBuff(tcpClient->readAll());

        qDebug() << qPrintable(dataBuff);






        std::vector<uint8_t> vec_buffer = std::vector<uint8_t>(dataBuff.begin(), dataBuff.end());
        EmitEvent([this,&vec_buffer](const ILinkEvents *ptr){ptr->ReceiveData(this, vec_buffer);});
  //  }

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
