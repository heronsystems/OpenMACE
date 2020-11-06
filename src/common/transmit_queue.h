#ifndef TRANSMIT_QUEUE_H
#define TRANSMIT_QUEUE_H

#include "thread_manager.h"

#include <mutex>
#include <functional>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include "common.h"

//!
//! \brief Stores an action to be executed on a seperate thread, and then reperformed if not dequeued.
//!
//! This object can hold things like Transmit task, that needs to be resent if the appropriate response is not heard.
//! The object can be tailored to reperform the tasks a specific number of times before a failure is invoked.
//!
//! The parameters for reexecution are set up in the constructor
//!
//! An action is queued by calling QueueTransmission, passing a lambda to do action and a lambda to execute upon failure.
//! That function is to return a number, which is to be used to dequeue the action using the RemoveTransmissionFromQueue method.
//!
//! Look at TransmitQueueWithKeys for a wrapper that can queue/dequeue actions based on a object (key).
//!
class TransmitQueue : public Thread
{
private:

    static const int DEFAULT_RESPONSE_WAIT_IN_MS = 2000000000;
    static const int DEFAULT_NUM_RETRIES = 3;

private:

    class TransmitTask
    {
    public:

        TransmitTask(std::function<void(int transmitNumber)> action, std::function<void(int transmitNumber)> failure, bool execute = true) :
            numTries(0),
            transmitAction(action),
            failureAction(failure),
            active(execute)
        {
        }

        int numTries;
        std::function<void(int transmitNumber)> transmitAction;
        std::function<void(int transmitNumber)> failureAction;
        std::chrono::time_point<std::chrono::system_clock> lastTransmit;
        bool active;
    };


private:

    std::mutex m_ActiveTransmitsMutex;
    std::unordered_map<int, TransmitTask> m_ActiveTransmits;

    int m_WaitForRetries;
    int m_NumRetries;


public:

    //!
    //! \brief Initialize a transmit queue
    //! \param waitForRetries Number of ms to wait before a retry is executed
    //! \param numRetries Number of retrys to execute before failing
    //!
    TransmitQueue(int waitForRetries = DEFAULT_RESPONSE_WAIT_IN_MS, int numRetries = DEFAULT_NUM_RETRIES)
    {
        m_WaitForRetries = waitForRetries;
        m_NumRetries = numRetries;

        if(waitForRetries > 1000000000)
        {
            printf("WARNING!!! Wait for retransmit is unreasonably high. Is is left on a debug value?\n");
        }
        start();
    }

    virtual ~TransmitQueue() = default;


    //!
    //! \brief Queue an action of some kind
    //!
    //! \param transmitAction Lambda indicating action to perform
    //! \param onFailure Lambda to call when the action fails. The action fails when the action hasn't been removed according to the parameters of the action
    //! \param execute [true] True if action is to be executed immediatly. If false action will not be performed until Execute is called.
    //! \return Number to identify the transmission, to be used to further reference the action.
    //!
    int QueueTransmission(std::function<void(int transmitNumber)> transmitAction, const std::function<void(int transmitNumber)> onFailure = [](int transmitNumber){UNUSED(transmitNumber);}, bool execute = true)
    {
        m_ActiveTransmitsMutex.lock();
        int num;
        do
        {
            num = std::rand();
        }
        while(this->m_ActiveTransmits.find(num) != m_ActiveTransmits.cend());

        m_ActiveTransmits.insert({num, TransmitTask(transmitAction, onFailure, execute)});
        m_ActiveTransmitsMutex.unlock();

        printf("Added Transmision - Number active: %u\n", (uint)m_ActiveTransmits.size());

        return num;
    }


    //!
    //! \brief Remove an action from queue
    //! \param ID Number that identifies the transmission.
    //!
    void RemoveTransmissionFromQueue(int ID)
    {
        m_ActiveTransmitsMutex.lock();
        if(m_ActiveTransmits.find(ID) != m_ActiveTransmits.cend())
        {
            m_ActiveTransmits.at(ID).active = false;
            m_ActiveTransmits.erase(ID);
        }
        m_ActiveTransmitsMutex.unlock();

        printf("Removed Transmission - Number active: %u\n", (uint)m_ActiveTransmits.size());
    }


    //!
    //! \brief Set an action to be executed
    //! \param num Number that identifies the transmission
    //!
    void Execute(int num)
    {
        if(m_ActiveTransmits.find(num) == m_ActiveTransmits.cend())
        {
            throw std::runtime_error("Given transmission number is unknown");
        }
        m_ActiveTransmits.at(num).active = true;
    }

    void run()
    {
        while(true)
        {
            if(isThreadActive() == false)
            {
                printf("!!!!!!!!!!!!SHUTTING DOWN TRANSMIT QUEUE!!!!!!\n");
                break;
            }

            if(m_ActiveTransmitsMutex.try_lock())
            {
                for(auto it = m_ActiveTransmits.begin() ; it != m_ActiveTransmits.end() ; ++it)
                {
                    if(it->second.active == false)
                    {
                        continue;
                    }
                    std::chrono::time_point<std::chrono::system_clock> currTime = std::chrono::system_clock::now();
                    int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(currTime - it->second.lastTransmit).count();
                    if(it->second.numTries == 0 || elapsed_ms > m_WaitForRetries)
                    {
                        //if we have exceeded number of retries then remove and fail out
                        if(it->second.numTries >= m_NumRetries)
                        {
                            it->second.failureAction(it->first);
                            it->second.active = false;
                            std::thread thread([this, it](){
                                RemoveTransmissionFromQueue(it->first);
                            });
                            thread.detach();
                            continue;
                        }

                        if(it->second.numTries >= 1)
                        {
                            // +1 to include original transmission
                            printf("Retransmitting for %d time\n", it->second.numTries+1);
                        }

                        it->second.lastTransmit = currTime;
                        it->second.numTries++;
                        it->second.transmitAction(it->first);


                    }
                }
                m_ActiveTransmitsMutex.unlock();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }


    }

    size_t Size()
    {
        return m_ActiveTransmits.size();
    }
};


//!
//! \brief Wrapper for a TransmitQueue that queues/deques transmissions indexed on external objects.
//!
//! Any number of external objects can be used to key transmissions.
//! Each object needs to impliment both == and < operators.
//!
//! The underlaying TransmissionQueue needs to be configured using SetQueue prior to use
//!
//! Transmissions are queued using QueueTransmission method.
//! There potentially multiple objects "keys" can be given that will be used to halt that transmission later
//! Additionally a lambda to perform the transmission is given, and a lambda for an action to perform when failed.
//!
//! The transmission action will execute, potentially mutliple times, according to what is set up with the TransmissionQueue given to this object.
//!
//! To dequeue a transmission RemoveTransmission is called with an object.
//! If that object matches any currently queued transmission, it will be removed.
//! If it doesn't match, nothing is done.
//!
//! This object is capable to queing transmissions to multiple targets or a single target
//! When queing to multiple targets, an internal list of target is kept.
//! Should the tranmission need to be resent, only the targets that haven't been removed will be provided
//! This allows the set up of a reliable broadcast
//!
//! \template COMPONENT_KEY Datatype to represent entities that will be targeted
//!
//! Usage:
//!
//! class A
//! {
//!     A(uint8_t id1, uint8_t id2) : ID1(id1), ID2(id2) {}
//!     uint8_t ID1
//!     uint8_t ID2
//! }
//!
//! class Entity
//! {
//!     int ID;
//! };
//!
//! TranmitQueue queue(2000, 4);
//! TransmitQueueWithKeys<Entity, int, A> keyWrapper;
//! keyWrapper.SetQueue(&queue);
//!
//! keyWrapper.QueueTransmissionToSingleTarget(1, [](){}, [](){});
//! keyWrapper.QueueTransmissionToSingleTarget(A(3,4), [](){}, [](){});
//!
//! keyWrapper.RemoveTransmission(A(3,4);
//! //This will only dequeue the second transmission added. After waiting 2000ms the first will be resent
//!
template <typename ...T>
class TransmitQueueWithKeys;


template <typename COMPONENT_KEY, typename Head, typename ...T>
class TransmitQueueWithKeys<COMPONENT_KEY, Head, T...> : public TransmitQueueWithKeys<COMPONENT_KEY, T...>
{
public:
    using TransmitQueueWithKeys<COMPONENT_KEY, T...>::QueueTransmission;
    using TransmitQueueWithKeys<COMPONENT_KEY, T...>::RemoveTransmission;
    using TransmitQueueWithKeys<COMPONENT_KEY, T...>::m_Queue;
    using TransmitQueueWithKeys<COMPONENT_KEY, T...>::m_ActiveTransmissionToTargets;
    using TransmitQueueWithKeys<COMPONENT_KEY, T...>::m_QueueManipulationMutex;

private:



    std::unordered_map<Head, int> m_ActiveTransmissions;

    std::unordered_map<int, std::vector<Head>> m_ActiveTransmissionsToKeyMap;

public:

    void QueueTransmission(const Head &key, const std::vector<COMPONENT_KEY> &targets, const std::function<void(const std::vector<COMPONENT_KEY> &)> &transmitAction, const std::function<void()> onFailure = [](){})
    {
        std::vector<Head> keys = {key};
        return QueueTransmission(keys, targets, transmitAction, onFailure);
    }

    void QueueTransmission(const std::vector<Head> &keys, const std::vector<COMPONENT_KEY> &targets, const std::function<void(const std::vector<COMPONENT_KEY> &)> &transmitAction, const std::function<void()> onFailure = [](){})
    {
        for(auto it = keys.cbegin() ; it != keys.cend() ; ++it)
        {
            if(m_ActiveTransmissions.find(*it) != m_ActiveTransmissions.cend())
            {
                printf("A Transmission with given end condition already queued. IGNORING\n");
                return;
            }
        }

        auto transmitFunction = [this, transmitAction](int transmitNumber){
            transmitAction(m_ActiveTransmissionToTargets.at(transmitNumber));
        };

        auto failFunction = [this, onFailure](int transmitNumber)
        {
            onFailure();
            m_QueueManipulationMutex.lock();
            for(auto it = m_ActiveTransmissionsToKeyMap.at(transmitNumber).cbegin() ; it != m_ActiveTransmissionsToKeyMap.at(transmitNumber).cend() ; ++it)
            {
                m_ActiveTransmissions.erase(*it);
            }
            m_ActiveTransmissionsToKeyMap.erase(transmitNumber);
            m_ActiveTransmissionToTargets.erase(transmitNumber);
            m_QueueManipulationMutex.unlock();
        };

        m_QueueManipulationMutex.lock();
        int num = m_Queue->QueueTransmission(transmitFunction, failFunction, false);
        for(auto it = keys.cbegin() ; it != keys.cend() ; ++it)
        {
            m_ActiveTransmissions.insert({*it, num});
        }
        if(m_ActiveTransmissionsToKeyMap.find(num) != m_ActiveTransmissionsToKeyMap.cend())
        {
            throw std::runtime_error("Need to set m_ActiveTransmissionsToKeyMap variable, but it already has an entry. Something got desynced");
        }
        if(m_ActiveTransmissionToTargets.find(num) != m_ActiveTransmissionToTargets.cend())
        {
            throw std::runtime_error("Need to set m_ActiveTransmissionToTargets variable, but it already has an entry. Something got desynced");
        }
        m_ActiveTransmissionsToKeyMap.insert({num, keys});
        m_ActiveTransmissionToTargets.insert({num, targets});
        m_QueueManipulationMutex.unlock();
        m_Queue->Execute(num);
    }

    void RemoveTransmission(const Head &key, OptionalParameter<COMPONENT_KEY> receivedFrom = OptionalParameter<COMPONENT_KEY>())
    {
        while(true)
        {
            if(m_ActiveTransmissions.find(key) != m_ActiveTransmissions.cend())
            {
                int num = m_ActiveTransmissions.at(key);
                bool removeTransmission = false;


                /// Check if user is using function properly
                if(m_ActiveTransmissionToTargets.find(num) != m_ActiveTransmissionToTargets.cend() && receivedFrom.IsSet() == false)
                {
                    printf("The transmission assosiated with the given key has a set of targets attributed to it, yet no received target given");
                }

                /// If we have been given an entity received from and the tranmission in question was distributed to multiple targets,
                ///   Then remove that target from the targets to send to. If that was the last target, then remove tranmission
                if(m_ActiveTransmissionToTargets.find(num) != m_ActiveTransmissionToTargets.cend() && receivedFrom.IsSet())
                {
                    m_ActiveTransmissionToTargets[num].erase(std::remove(m_ActiveTransmissionToTargets[num].begin(), m_ActiveTransmissionToTargets[num].end(), receivedFrom.Value()), m_ActiveTransmissionToTargets[num].end());

                    if(m_ActiveTransmissionToTargets[num].size() == 0)
                    {
                        removeTransmission = true;
                    }
                }


                /// If the transmission was NOT distributed to multiple targets, then set flag to remove tranmission
                if(m_ActiveTransmissionToTargets.find(num) == m_ActiveTransmissionToTargets.cend())
                {
                    removeTransmission = true;
                }


                /// Remove Transmission
                if(removeTransmission == true)
                {
                    m_QueueManipulationMutex.lock();
                    m_Queue->RemoveTransmissionFromQueue(num);
                    if(m_ActiveTransmissionsToKeyMap.find(num) != m_ActiveTransmissionsToKeyMap.cend()) {
                        for(auto it = m_ActiveTransmissionsToKeyMap.at(num).cbegin() ; it != m_ActiveTransmissionsToKeyMap.at(num).cend() ; ++it)
                        {
                            m_ActiveTransmissions.erase(*it);
                        }
                    }
                    m_ActiveTransmissionsToKeyMap.erase(num);
                    m_ActiveTransmissionToTargets.erase(num);
                    m_QueueManipulationMutex.unlock();
                    continue;
                }
            }
            break;
        }
    }

    /*!
     * \brief Removes all queued transmissions.
     */
    void RemoveAllTransmissions()
    {
        m_QueueManipulationMutex.lock();
        int num;
        if (!m_ActiveTransmissions.empty())
        {
            auto it = m_ActiveTransmissions.begin();
            while (it != m_ActiveTransmissions.end())
            {
                num = it->second;
                m_Queue->RemoveTransmissionFromQueue(num);
                ++it;
            }
        }
        m_ActiveTransmissions.clear();
        m_ActiveTransmissionsToKeyMap.clear();
        m_QueueManipulationMutex.unlock();
    }
};

template <typename COMPONENT_KEY>
class TransmitQueueWithKeys<COMPONENT_KEY>
{
protected:

    TransmitQueue* m_Queue;
    std::mutex m_QueueManipulationMutex;
    std::unordered_map<int, std::vector<COMPONENT_KEY>> m_ActiveTransmissionToTargets;

public:

    virtual ~TransmitQueueWithKeys() = default;

    void SetQueue(TransmitQueue* queue) {
        m_Queue = queue;
    }

    void QueueTransmission()
    {

    }

    void RemoveTransmission()
    {

    }

    void RemoveAllTransmissions()
    {

    }
};


#endif // TRANSMIT_QUEUE_H
