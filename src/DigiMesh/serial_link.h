#ifndef SERIALLINK_H
#define SERIALLINK_H

#include "DigiMesh_global.h"
#include <chrono>
#include <mutex>
#include <iostream>
#include <thread>

#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QThread>

#include <functional>

#include "serial_configuration.h"

#include "i_link_events.h"

class DIGIMESHSHARED_EXPORT SerialLink
{
private:

    std::vector<ILinkEvents*> m_Listeners;

public:

    SerialLink(const SerialConfiguration &config);

    ~SerialLink();

    void AddListener(ILinkEvents* ptr)
    {
        m_Listeners.push_back(ptr);
    }

    void EmitEvent(const std::function<void(ILinkEvents*)> &func)
    {
        for(ILinkEvents* listener : m_Listeners)
        {
            func(listener);
        }
    }

    virtual void RequestReset();

    virtual void WriteBytes(const char *bytes, int length);

    //!
    //! \brief Determine the connection status
    //! \return True if the connection is established, false otherwise
    //!
    virtual bool isConnected() const;


    std::string getPortName() const;

    void _emitLinkError(const std::string& errorMsg);

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

#endif // SERIALLINK_H
