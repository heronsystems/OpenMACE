#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <thread>

#include <functional>
#include <list>
#include <thread>
#include <mutex>

class Thread
{
public:
    Thread() : mThread(nullptr), mToExit(false)
    {

    }

    virtual ~Thread()
    {
        stop();
    }

    virtual void run() = 0;

    virtual void start()
    {
        stop();
        mToExit = false;
        mThread = new std::thread([this]() {
            this->run();
        });
    }

    void stop()
    {
        if (mThread)
        {
            mToExit = true;
            mThread->join();
            delete mThread;
            mThread = nullptr;
        }
    }

    bool isThreadActive()
    {
        if (mToExit == false)
        {
            return true;
        }
        return false;
    }

protected:
    std::list<std::function<void()>> m_LambdasToRun;

    virtual void clearPendingTasks()
    {
        _lambdaMutex.lock();
        m_LambdasToRun.clear();
        _lambdaMutex.unlock();
    }

    virtual void RunPendingTasks()
    {
        _lambdaMutex.lock();
        while (m_LambdasToRun.size() > 0)
        {
            auto lambda = m_LambdasToRun.front();
            m_LambdasToRun.pop_front();
            lambda();
        }
        _lambdaMutex.unlock();
    }

protected:
    std::recursive_mutex _lambdaMutex;
    std::thread *mThread;
    bool mToExit;
};

#endif // THREADMANAGER_H
