#ifndef BACKGROUND_TASKS_H
#define BACKGROUND_TASKS_H

#include <mutex>
#include <thread>

template <typename T>
class BackgroundTasks
{
private:

    std::mutex m_mutex;
    std::thread *m_thread;
    std::function<void(const T &data)> m_Action;
    bool m_ActionSet;

    bool m_HasNextData;
    T m_NextData;

public:

    BackgroundTasks() :
        m_ActionSet(false),
        m_HasNextData(false)
    {
        m_thread = new std::thread([this](){this->run();});
    }

    BackgroundTasks(std::function<void(const T &data)> action) :
        m_Action(action),
        m_ActionSet(true),
        m_HasNextData(false)
    {
        m_thread = new std::thread([this](){this->run();});
    }

    ~BackgroundTasks()
    {
        delete m_thread;
    }

    void SetAction(std::function<void(const T &data)> action)
    {
        m_Action = action;
        m_ActionSet = true;
    }

    void NewTasks(const T &data)
    {
        if(m_ActionSet == false)
        {
            throw std::runtime_error("No action given");
        }
        m_mutex.lock();
        m_NextData = data;
        m_HasNextData = true;
        m_mutex.unlock();
    }


private:

    void run()
    {
        while(true)
        {
            if(m_HasNextData == true)
            {
                m_mutex.lock();
                T data = m_NextData;
                m_HasNextData = false;
                m_mutex.unlock();

                m_Action(data);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};

#endif // BACKGROUND_TASKS_H
