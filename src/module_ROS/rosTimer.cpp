#include "rosTimer.h"

ROSTimer::ROSTimer(const Timeout &timeout)
    : _timeout(timeout)
{
}

ROSTimer::ROSTimer(const ROSTimer::Timeout &timeout,
             const ROSTimer::Interval &interval,
             bool singleShot)
    : _isSingleShot(singleShot),
      _interval(interval),
      _timeout(timeout)
{
}

void ROSTimer::start(bool multiThread)
{
    if (this->running() == true)
        return;

    _running = true;

    if (multiThread == true) {
        _thread = std::thread(
                    &ROSTimer::_temporize, this);
    }
    else{
        this->_temporize();
    }
}

void ROSTimer::stop()
{
    _running = false;
    _thread.join();
}

bool ROSTimer::running() const
{
    return _running;
}

void ROSTimer::setSingleShot(bool singleShot)
{
    if (this->running() == true)
       return;

    _isSingleShot = singleShot;
}

bool ROSTimer::isSingleShot() const
{
    return _isSingleShot;
}

void ROSTimer::setInterval(const ROSTimer::Interval &interval)
{
    if (this->running() == true)
       return;

    _interval = interval;
}

const ROSTimer::Interval &ROSTimer::interval() const
{
    return _interval;
}

void ROSTimer::setTimeout(const Timeout &timeout)
{
    if (this->running() == true)
       return;

    _timeout = timeout;
}

const ROSTimer::Timeout &ROSTimer::timeout() const
{
    return _timeout;
}

void ROSTimer::_temporize()
{
    if (_isSingleShot == true) {
        this->_sleepThenTimeout();
    }
    else {
        while (this->running() == true) {
            this->_sleepThenTimeout();
        }
    }
}

void ROSTimer::_sleepThenTimeout()
{
    std::this_thread::sleep_for(_interval);

    if (this->running() == true)
        this->timeout()();
}
