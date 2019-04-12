#ifndef TCPLINK_MACE_H
#define TCPLINK_MACE_H

#include "commsmace_global.h"

#include <mutex>
#include <iostream>
#include <thread>
#include <map>

#include <QCoreApplication>
#include <QThread>
#include <QTcpSocket>
#include <QTcpServer>
#include <QUdpSocket>

#include "tcp_configuration_mace.h"

#include "i_link_mace.h"


namespace CommsMACE
{



class COMMSMACESHARED_EXPORT TcpLink : public ILink
{

public:

    TcpLink(const TcpConfiguration &config);

    ~TcpLink();

    virtual void RequestReset();
    virtual uint64_t getConnectionSpeed() const;

    virtual void WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());
    void WriteBytesBroadcast(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>()) const;
    virtual void AddResource(const Resource &resource);

    virtual bool HasResource(const Resource &resource) const;

    virtual void RequestRemoteResources() const;
    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;

    std::string getTCPServerAddress() const;
    int getTCPServerPort() const;
    int getUDPBroadcastPort() const;



    void _emitLinkError(const std::string& errorMsg) const;

    LinkConfiguration getLinkConfiguration();



    virtual bool Connect(void);

    virtual void Disconnect(void);

    virtual void MarshalOnThread(std::function<void()> func){
        ///////////////////
        /// Determine what thread to run function on
        QThread *threadToMashalOn = serverThread;
        QThread *currentThread = QThread::currentThread();

        //the current thread is the thread that link operates on
        if(threadToMashalOn == currentThread)
        {
            func();
        }
        else {
            postToThread([func](){
                func();
            }, m_udpSocket);
        }
    }

    bool StartTCPServer();
    bool StartUDPListener();
    void startTCPClient();

    // ============================================================================= //
    // =============================== Public slots ================================ //
    // ============================================================================= //
public slots:
    //!
    //! \brief on_newConnection Slot to fire when a new TCP connection is initiated
    //!
    void on_newConnection();

private:

    template <typename F>
    static void postToThread(F && fun, QObject * obj = qApp) {
      struct Event : public QEvent {
        F fun;
        Event(F && fun) : QEvent(QEvent::None), fun(std::move(fun)) {}
        ~Event() {
            fun();
        }
      };
      QCoreApplication::postEvent(obj, new Event(std::move(fun)));
    }


private:


    void processPendingDatagrams(void);
    void processData(void);

    void linkError(QTcpSocket::SocketError error);

private:

    QTcpSocket* tcpClient;
    std::shared_ptr<QTcpServer> tcpServer;
    QTcpSocket* tcpServerSocket;
    bool serverStarted = false;
    quint64 m_bytesRead;
    int     m_timeout;
    std::thread *m_CommsThread;
    QThread *serverThread;
    QThread *m_UDPListenThread;
    std::vector<QThread*> clientThreads;
    std::mutex  m_dataMutex;       // Mutex for reading data from _port
    std::mutex  m_writeMutex;      // Mutex for accessing the _transmitBuffer.

    QUdpSocket* m_udpSocket;
    std::map<const qintptr, QTcpSocket*> clientMap;


    volatile bool       m_stopp;
    volatile bool       m_reqReset;
    std::mutex          m_stoppMutex;      // Mutex for accessing _stopp
    TcpConfiguration    _config;
};

}


#endif // UDPLINK_H
