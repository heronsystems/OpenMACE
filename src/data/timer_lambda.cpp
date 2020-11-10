#include "timer_lambda.h"

TimerLambda::TimerLambda(const unsigned int &timeout)
{
    _timeout = timeout;
}

TimerLambda::~TimerLambda()
{

}

void TimerLambda::start()
{
    m_Timeout.start();
    Thread::start();
}

void TimerLambda::RemoveHost(void *ptr)
{
    _mutex_Timeout.lock();
    m_TimeoutLambda.erase(ptr);
    _mutex_Timeout.unlock();
}

void TimerLambda::setLambda_Timeout(const std::function<void()> &lambda)
{
    _mutex_Timeout.lock();
    if (m_TimeoutLambda.find(0) != m_TimeoutLambda.cend())
        m_TimeoutLambda.erase(0);
    m_TimeoutLambda.insert({0, lambda});
    _mutex_Timeout.unlock();
}

void TimerLambda::addLambda_Timeout(void *host, const std::function<void()> &lambda)
{
    _mutex_Timeout.lock();
    m_TimeoutLambda.insert({host, lambda});
    _mutex_Timeout.unlock();
}

void TimerLambda::on_Timeout()
{
    _mutex_Timeout.lock();
    for (auto it = m_TimeoutLambda.cbegin(); it != m_TimeoutLambda.cend(); ++it)
        it->second();
    _mutex_Timeout.unlock();
}

void TimerLambda::run()
{
    while(true)
    {
        if(mToExit == true) {
            clearPendingTasks();
            m_Timeout.stop();
            break;
        }

        this->RunPendingTasks();

        //The current state we can find out how much time has passed.
        //If one of the lambda expressions has fired the clock should
        //be reset right at the end, thus making this value small and
        //improbable the next function will fire
        double timeElapsed = m_Timeout.elapsedMilliseconds();

        if(timeElapsed >= _timeout)
        {
            on_Timeout();
            m_Timeout.reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(_timeout/2));
    }
}
