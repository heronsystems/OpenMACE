#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <thread>
#include <time.h>
#include <functional>

class Watchdog
{
private:

    std::chrono::duration<double> m_duration;
    std::function<void()> m_Lambda;
    std::thread m_thread;
    std::chrono::high_resolution_clock::time_point m_lastKick;
    bool m_Finished;
    bool m_CountinuouslyCall;

public:
    Watchdog(const std::chrono::duration<double> &duration, const std::function<void()> &lambda) :
        m_duration(duration),
        m_Lambda(lambda),
        m_thread([this](){this->run();}),
        m_Finished(false),
        m_CountinuouslyCall(false)
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }

    ~Watchdog()
    {
        m_Finished = true;
        m_thread.join();
    }


    void Kick()
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }


private:

    void run()
    {
        while(true)
        {
            if(m_Finished) {
                break;
            }
            std::chrono::duration<double> durationSinceLastKick = std::chrono::high_resolution_clock::now() - m_lastKick;
            if(durationSinceLastKick > m_duration)
            {
                m_Lambda();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};





class ContinuousWatchdog
{
private:

    std::chrono::duration<double> m_duration;
    std::function<bool()> m_Lambda;
    std::thread m_thread;
    std::chrono::high_resolution_clock::time_point m_lastKick;
    bool m_Finished;
    bool m_CountinuouslyCall;

public:

    ContinuousWatchdog(const std::chrono::duration<double> &duration, const std::function<bool()> &lambda) :
        m_duration(duration),
        m_Lambda(lambda),
        m_thread([this](){this->run();}),
        m_Finished(false),
        m_CountinuouslyCall(true)
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }

    ~ContinuousWatchdog()
    {
        m_Finished = true;
        m_thread.join();
    }


    void Kick()
    {
        m_lastKick = std::chrono::high_resolution_clock::now();
    }

private:

    void run()
    {
        while(true)
        {
            if(m_Finished) {
                break;
            }
            std::chrono::duration<double> durationSinceLastKick = std::chrono::high_resolution_clock::now() - m_lastKick;
            if(durationSinceLastKick > m_duration)
            {
                bool isDone = m_Lambda();
                if(isDone == true)
                {
                    m_Finished = true;
                }
                else
                {
                    Kick();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

#endif // WATCHDOG_H
