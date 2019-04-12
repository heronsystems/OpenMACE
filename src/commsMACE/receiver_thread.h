#ifndef RECEIVER_THREAD_H
#define RECEIVER_THREAD_H

#include <QThread>
#include <QCoreApplication>


#ifdef WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <unistd.h>
//void Sleep(int sleepMs)
//{
//    usleep(sleepMs * 1000);   // usleep takes sleep time in us (1 millionth of a second)
//}
#endif

namespace CommsMACE {

//!
//! \brief This class defines a thread such that a QObject can run in peace.
//!
class ReceiverThread : public QThread
{
public:
    ReceiverThread(const std::function<void(void)> &func, uint32_t defaultWait = 0):
        m_func(func),
        m_defaultWait(defaultWait)
    {
        if(QCoreApplication::instance() == NULL)
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
            QCoreApplication::processEvents();
            m_func();

            if(m_defaultWait > 0) {
#ifdef WIN32
                Sleep(m_defaultWait);
#elif __linux__
                usleep(m_defaultWait * 1000);   // usleep takes sleep time in us (1 millionth of a second)
#endif
            }
        }
    }

private:

    std::function<void(void)> m_func;
    QCoreApplication *pApp;
    uint32_t m_defaultWait;
};


}

#endif // RECEIVER_THREAD_H
