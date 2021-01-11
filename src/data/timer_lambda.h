/*!
  * @file timer_lambda.h
  *
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *
  * @section DESCRIPTION
  *
  * @date
  *     Feb 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#ifndef TIMER_LAMBDA_H
#define TIMER_LAMBDA_H

#include <functional>
#include <mutex>
#include <unordered_map>

#include "common/class_forward.h"
#include "common/thread_manager.h"
#include "timer.h"

class TimerLambda : public Thread
{
public:
    TimerLambda(const unsigned int &timeout = 1000); //timeout should be in milliseconds

    ~TimerLambda() override;

public: 
    void start() override;

    void run() override;

public:
    virtual void RemoveHost(void *ptr);

    void setLambda_Timeout(const std::function<void()> &lambda);

    void addLambda_Timeout(void *host, const std::function<void()> &lambda);

    void on_Timeout();

private:
    std::mutex _mutex_Timeout;
    std::unordered_map<void *, std::function<void()>> m_TimeoutLambda;

private: 
    Timer m_Timeout;

    unsigned int _timeout = 0;
};

#endif // TIMER_LAMBDA_H
