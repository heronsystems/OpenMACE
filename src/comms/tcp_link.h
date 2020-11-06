#ifndef TCPLINK_H
#define TCPLINK_H

#include "comms_global.h"

#include <mutex>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QThread>
#include <QTcpSocket>
#include <QTcpServer>

#include "tcp_configuration.h"

#include "i_link.h"

#include "common/common.h"


namespace Comms
{



class COMMSSHARED_EXPORT TcpLink : public ILink
{
public:

    TcpLink(const TcpConfiguration &config);

    ~TcpLink();

    virtual void RequestReset();
    virtual uint64_t getConnectionSpeed() const;

    virtual void WriteBytes(const char *bytes, int length) const;

    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;

    std::string getListenAddress() const;
    int getListenPortNumber() const;
    std::string getSenderAddress() const;
    int getSenderPortNumber() const;



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
            }, tcpClient);
        }
    }

    bool StartTCPServer();
    void startTCPClient();

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


    /// Performs the actual hardware port connection.
    ///     @param[out] error if failed
    ///     @param[out] error string if failed
    /// @return success/fail
    bool _hardwareConnect(QAbstractSocket::SocketError& error, QString& errorString);

private:


    void processData(void);

    void linkError(QTcpSocket::SocketError error);

private:

    QTcpSocket* tcpClient;
    QTcpServer* tcpServer;
    quint64 m_bytesRead;
    int     m_timeout;
    std::thread *m_CommsThread;
    QThread *serverThread;
    QThread *clientThread;
    std::mutex  m_dataMutex;       // Mutex for reading data from _port
    std::mutex  m_writeMutex;      // Mutex for accessing the _transmitBuffer.


    volatile bool        m_stopp;
    volatile bool        m_reqReset;
    std::mutex           m_stoppMutex;      // Mutex for accessing _stopp
    TcpConfiguration _config;
};

}

#endif // TCPLINK_H
