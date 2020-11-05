#include "mltimer.h"

MLTimer::MLTimer(const Timeout &timeout)
    : _timeout(timeout)
{
}

MLTimer::MLTimer(const MLTimer::Timeout &timeout,
             const MLTimer::Interval &interval,
             bool singleShot)
    : _isSingleShot(singleShot),
      _interval(interval),
      _timeout(timeout)
{
}

void MLTimer::start(bool multiThread)
{
    if (this->running() == true)
        return;

    _running = true;

    if (multiThread == true) {
        _thread = std::thread(
                    &MLTimer::_temporize, this);
    }
    else{
        this->_temporize();
    }
}

void MLTimer::stop()
{
    _running = false;
    _thread.join();
}

bool MLTimer::running() const
{
    return _running;
}

void MLTimer::setSingleShot(bool singleShot)
{
    if (this->running() == true)
       return;

    _isSingleShot = singleShot;
}

bool MLTimer::isSingleShot() const
{
    return _isSingleShot;
}

void MLTimer::setInterval(const MLTimer::Interval &interval)
{
    if (this->running() == true)
       return;

    _interval = interval;
}

const MLTimer::Interval &MLTimer::interval() const
{
    return _interval;
}

void MLTimer::setTimeout(const Timeout &timeout)
{
    if (this->running() == true)
       return;

    _timeout = timeout;
}

const MLTimer::Timeout &MLTimer::timeout() const
{
    return _timeout;
}

void MLTimer::_temporize()
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

void MLTimer::_sleepThenTimeout()
{
    std::this_thread::sleep_for(_interval);

    if (this->running() == true)
        this->timeout()();
}
