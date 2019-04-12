#include "guitimer.h"

GUITimer::GUITimer(const Timeout &timeout)
    : _timeout(timeout)
{
}

GUITimer::GUITimer(const GUITimer::Timeout &timeout,
             const GUITimer::Interval &interval,
             bool singleShot)
    : _isSingleShot(singleShot),
      _interval(interval),
      _timeout(timeout)
{
}

void GUITimer::start(bool multiThread)
{
    if (this->running() == true)
        return;

    _running = true;

    if (multiThread == true) {
        _thread = std::thread(
                    &GUITimer::_temporize, this);
    }
    else{
        this->_temporize();
    }
}

void GUITimer::stop()
{
    _running = false;
    _thread.join();
}

bool GUITimer::running() const
{
    return _running;
}

void GUITimer::setSingleShot(bool singleShot)
{
    if (this->running() == true)
       return;

    _isSingleShot = singleShot;
}

bool GUITimer::isSingleShot() const
{
    return _isSingleShot;
}

void GUITimer::setInterval(const GUITimer::Interval &interval)
{
    if (this->running() == true)
       return;

    _interval = interval;
}

const GUITimer::Interval &GUITimer::interval() const
{
    return _interval;
}

void GUITimer::setTimeout(const Timeout &timeout)
{
    if (this->running() == true)
       return;

    _timeout = timeout;
}

const GUITimer::Timeout &GUITimer::timeout() const
{
    return _timeout;
}

void GUITimer::_temporize()
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

void GUITimer::_sleepThenTimeout()
{
    std::this_thread::sleep_for(_interval);

    if (this->running() == true)
        this->timeout()();
}
