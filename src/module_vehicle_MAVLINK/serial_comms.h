#ifndef SERIALCOMMS_H
#define SERIALCOMMS_H

#include <QSerialPort>
#include <thread>

class SerialComms : private QSerialPort
{
public:
    SerialComms() :
        m_ThreadRunning(true),
        m_ReadThread([this](){this->readLoop();})
    {

    }

private:

    void readLoop()
    {
        while(m_ThreadRunning)
        {
            if(waitForReadyRead(10) == false)
            {
                m_ThreadRunning = false;
                break;
            }


        }
    }


private:


    bool m_ThreadRunning;
    std::thread m_ReadThread;


};

#endif // SERIALCOMMS_H
