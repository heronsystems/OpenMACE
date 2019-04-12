#ifndef SERIALLINK_MACE_H
#define SERIALLINK_MACE_H

#include "commsmace_global.h"

#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QThread>

#include "serial_configuration_mace.h"

#include "i_link_mace.h"

namespace CommsMACE
{



class COMMSMACESHARED_EXPORT SerialLink : public ILink
{
public:

    SerialLink(const SerialConfiguration &config);

    ~SerialLink();

    virtual void RequestReset();

    virtual void WriteBytes(const char *bytes, int length, const OptionalParameter<Resource> &target = OptionalParameter<Resource>());

    virtual void AddResource(const Resource &resource);

    virtual bool HasResource(const Resource &resource) const;

    virtual void RequestRemoteResources() const;


    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;


    std::string getPortName() const;


    //!
    //! \brief Get the maximum connection speed for this interface.
    //! \return The nominal data rate of the interface in bit per second, 0 if unknown
    //!
    virtual uint64_t getConnectionSpeed() const;



    void _emitLinkError(const std::string& errorMsg) const;


    LinkConfiguration getLinkConfiguration();



    virtual bool Connect(void);

    virtual void Disconnect(void);

    virtual void MarshalOnThread(std::function<void()> func){
        ///////////////////
        /// Determine what thread to run function on
        QThread *threadToMashalOn = m_ListenThread;
        QThread *currentThread = QThread::currentThread();

        //the current thread is the thread that link operates on
        if(threadToMashalOn == currentThread)
        {
            func();
        }
        else {
            postToThread([func](){
                func();
            }, m_port);
        }
    }

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
    bool _hardwareConnect(QSerialPort::SerialPortError& error, QString& errorString);


    bool _isBootloader();

private:


    void _readBytes(void);

    void linkError(QSerialPort::SerialPortError error);

private:

    void PortEventLoop();


private:

    QSerialPort* m_port;
    quint64 m_bytesRead;
    int     m_timeout;
    QThread *m_ListenThread;
    std::mutex  m_dataMutex;       // Mutex for reading data from _port
    std::mutex  m_writeMutex;      // Mutex for accessing the _transmitBuffer.


    volatile bool        m_stopp;
    volatile bool        m_reqReset;
    std::mutex           m_stoppMutex;      // Mutex for accessing _stopp
    SerialConfiguration _config;
};

} //END MAVLINKComms

#endif // SERIALLINK_H
