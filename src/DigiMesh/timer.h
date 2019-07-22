#ifndef TIMER_H
#define TIMER_H

#include <functional>
#include <chrono>
#include <thread>

class Timer
{
private:
    std::function<void()> m_Func;
    std::thread m_Thread;
public:
    Timer(int timeoutInMS, const std::function<void()> func) {
        m_Func = func;

        m_Thread = std::thread([timeoutInMS, func]() {
           std::this_thread::sleep_for(std::chrono::milliseconds(timeoutInMS));
           func();
        });
    }
};

#endif // TIMER_H
