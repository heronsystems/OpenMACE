/*!
  * @file queue_interface.h
  * 
  * @authors
  *     Kenneth Kroeger ken.kroeger@heronsystems.com
  *     Patrick Nolan pat.nolan@heronsystems.com
  *
  * @section PROJECT
  *     This is a part of Heron Systems participation within the APL's Skyborg Program exercising TACE with existing autonomy. 
  *
  * @section DESCRIPTION
  *
  * @date
  *     March 2020
  *
  * @copyright
  *     File and its related contents are subjected to a proprietary software license from
  *     Heron Systems Inc. The extent of the rights and further details are located in the
  *     top level of directory via LICENSE.md file.
  **/

#ifndef QUEUE_INTERFACE_H
#define QUEUE_INTERFACE_H

#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

class QueueInterface
{
public:
    QueueInterface() = default;

public:
    virtual void RemoveHost(void *ptr)
    {
        _mutex_QueueUpdate.lock();
        m_QueueUpdatedLambda.erase(ptr);
        _mutex_QueueUpdate.unlock();
    }

    void setLambda_QueueUpdate(const std::function<void()> &lambda)
    {
        _mutex_QueueUpdate.lock();
        if (m_QueueUpdatedLambda.find(0) != m_QueueUpdatedLambda.cend())
            m_QueueUpdatedLambda.erase(0);
        m_QueueUpdatedLambda.insert({0, lambda});
        _mutex_QueueUpdate.unlock();
    }

    void addLambda_QueueUpdate(void *host, const std::function<void()> &lambda)
    {
        _mutex_QueueUpdate.lock();
        m_QueueUpdatedLambda.insert({host, lambda});
        _mutex_QueueUpdate.unlock();
    }

    void on_QueueUpdate()
    {
        _mutex_QueueUpdate.lock();
        for (auto it = m_QueueUpdatedLambda.cbegin(); it != m_QueueUpdatedLambda.cend(); ++it)
            it->second();
        _mutex_QueueUpdate.unlock();
    }

protected:
    std::mutex _mutex_QueueUpdate;
    std::unordered_map<void *, std::function<void()>> m_QueueUpdatedLambda;
};

#endif // QUEUE_INTERFACE_H