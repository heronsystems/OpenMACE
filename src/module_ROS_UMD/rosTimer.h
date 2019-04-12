#ifndef ROSTIMER_H
#define ROSTIMER_H

#include <thread>
#include <chrono>

#include <functional>

class ROSTimer
{
public:
    typedef std::chrono::milliseconds Interval;
    typedef std::function<void(void)> Timeout;

    ROSTimer(const Timeout &timeout);
    ROSTimer(const Timeout &timeout,
          const Interval &interval,
          bool singleShot = true);

    void start(bool multiThread = false);
    void stop();

    bool running() const;

    void setSingleShot(bool singleShot);
    bool isSingleShot() const;

    void setInterval(const Interval &interval);
    const Interval &interval() const;

    void setTimeout(const Timeout &timeout);
    const Timeout &timeout() const;

private:
    std::thread _thread;

    bool _running = false;
    bool _isSingleShot = true;

    Interval _interval = Interval(0);
    Timeout _timeout = nullptr;

    void _temporize();
    void _sleepThenTimeout();
};

#endif // ROSTIMER_H
