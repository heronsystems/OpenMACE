#ifndef GENERIC_MACE_CONTROLLER_H
#define GENERIC_MACE_CONTROLLER_H

#include "common/common.h"

#include <unordered_map>

#include "mace_core/module_characteristics.h"
#include "I_controller.h"
#include "common/pointer_collection.h"

#include <tuple>
#include <functional>

#include "common/optional_parameter.h"
#include "common/chain_inheritance.h"
#include "common/object_int_tuple.h"

#include "I_message_notifier.h"

#include "base_data_item.h"

#include "common/transmit_queue.h"
#include "mace_core/module_characteristics.h"

namespace Controllers {


//!
//! \brief Sets up basic operations of a communications controller
//!
//! A communications controller is an object that exposes a reliable communication strategy on a potentially unreliable communication paradigm.
//! A single controller is to be used to control communication exchange for a single _concept_
//! For example a "Boundary Controller" may be used to manage all messages that are needed to transmit a boundary
//!
//! Set up of a controllers actions is to be drivin by associating various actions when building the concrete controller.
//! See examples for how this is done.
//!
//! When the controller is done with its action, it will call a lambda given in setLambda_Finished method.
//! This lambda will be called with a boolean indicating if controller finished or failed, and a FINISH_CODE value.
//! The FINISH_CODE is a templated ack-code that is specific on the controllers use.
//!
//!
//! \template MESSAGETYPE The underlaying message type that is sent to the communication link. On MAVLINK: mavlink_message_t, In MACE: mace_message_t
//! \template COMPONENT_KEY Key that identifies entities communicating on the communication link. On MAVLINK this is a vehicleID, in MACE this is set to ModuleCharacterstic.
//! \template TransmitQueueType Type of TransmitQueueWithKeys indicating what objects this controller will use to queue transmissions with
//! \template FINISH_CODE type that is to be returned when done
//! \template DataItems Set of data items that the controller is to have as input, or generate as output.
//!
template<typename MESSAGETYPE, typename COMPONENT_KEY, typename TransmitQueueType, typename FINISH_CODE, typename ...DataItems>
class GenericController : public IController<MESSAGETYPE, COMPONENT_KEY>, public TransmitQueueType, public ChainInheritance<DataItems...>
{
private:

    int m_LinkChan;

    const IMessageNotifier<MESSAGETYPE, COMPONENT_KEY>* m_CB;

protected:

    std::vector<std::tuple<
        std::function<bool(COMPONENT_KEY, const MESSAGETYPE*)>,
        std::function<void(COMPONENT_KEY, const MESSAGETYPE*)>
    >> m_MessageBehaviors;

    std::mutex m_MessageBehaviorsMutex;

    std::unordered_map<void*, std::function<void(const bool completed, const FINISH_CODE finishCode)>> m_FinishLambda;
    std::unordered_map<void*, std::function<void()>> m_ShutdownLambda;

    std::mutex m_MutexFinishLambda;
    std::mutex m_MutexShutdownLambda;

public:

    //!
    //! \brief Contructor
    //! \param cb Pointer to object that is to operate the controller. Needed to make callbacks on
    //! \param queue Pointer to queue object, A single queue should be shared among all controllers
    //! \param linkChan Chanel communication's are to operate on.
    //!
    GenericController(const IMessageNotifier<MESSAGETYPE, COMPONENT_KEY>* cb, TransmitQueue* queue, int linkChan) :
        m_LinkChan(linkChan),
        m_CB(cb)
    {
        TransmitQueueType::SetQueue(queue);
    }

    virtual ~GenericController() = default;

    //!
    //! \brief Remove all action tied to the given host
    //! \param ptr Pointer to host that actions attributed to are to be removed
    //!
    virtual void RemoveHost(void* ptr)
    {
        m_MutexFinishLambda.lock();
        m_FinishLambda.erase(ptr);
        m_MutexFinishLambda.unlock();

        m_MutexShutdownLambda.lock();
        m_ShutdownLambda.erase(ptr);
        m_MutexShutdownLambda.unlock();
    }

    void setLambda_Finished(const std::function<void(const bool completed, const FINISH_CODE finishCode)> &lambda){
        m_MutexFinishLambda.lock();

        if(m_FinishLambda.find(0) != m_FinishLambda.cend())
        {
            printf("Warning!!!! A finish procedure already exists, replacing old with new\n");
            m_FinishLambda.erase(0);
        }

        m_FinishLambda.insert({0, lambda});
        m_MutexFinishLambda.unlock();
    }

    //!
    //! \brief Add a behavior to do when controller is finished
    //!
    //! This behavior is tied to a "host", which can be removed should the controller persist beyond the host
    //!
    //! \param host Pointer to host of behavior
    //! \param lambda Action to perform
    //!
    void AddLambda_Finished(void* host, const std::function<void(const bool completed, const FINISH_CODE finishCode)> &lambda){
        m_MutexFinishLambda.lock();
        m_FinishLambda.insert({host, lambda});
        m_MutexFinishLambda.unlock();
    }



    void onFinished(const bool completed, const FINISH_CODE finishCode = FINISH_CODE()){

        m_MutexFinishLambda.lock();
        for(auto it = m_FinishLambda.cbegin() ; it != m_FinishLambda.cend() ; ++it)
        {
            it->second(completed, finishCode);
        }
        m_MutexFinishLambda.unlock();
    }

    //!
    //! \brief Sets a lambda to perform some shutdown action.
    //!
    //! The shutdown action will be performed on its own thread when Shutdown is called.
    //! See Shutdown method for more discussion on this behavior.
    //! \param lambda Lambda to set
    //!
    void setLambda_Shutdown(const std::function<void()> &lambda){

        if(m_ShutdownLambda.find(0) != m_ShutdownLambda.cend())
        {
            printf("Warning!!!! A shutdown procedure already exists, replacing old with new\n");
            m_ShutdownLambda[0] = lambda;
        }
        else
        {
            m_ShutdownLambda.insert({0, lambda});
        }


    }


    //!
    //! \brief Sets a lambda to perform some shutdown action.
    //!
    //! The shutdown action will be performed on its own thread when Shutdown is called.
    //! See Shutdown method for more discussion on this behavior.
    //! \param lambda Lambda to set
    //!
    void AddLambda_Shutdown(void* sender, const std::function<void()> &lambda){
        m_MutexShutdownLambda.lock();
        m_ShutdownLambda.insert({sender, lambda});
        m_MutexShutdownLambda.unlock();
    }


    //!
    //! \brief Shutdown the controller.
    //!
    //! Will call the lambda set by setLambda_Shutdown on a seperate thread.
    //! This allows a mutex to lock out resources that will probably be locked when other lambdas are called.
    //!   i.e. if the controller is removed from a list when onFinished is called, any mutex protecting that removal will probably already be locked from receiving.
    //!
    //! Any previously queued transmissions will be removed as well.
    //!
    void Shutdown()
    {
        std::thread thread([this](){
            this->RemoveAllTransmissions();
            this->onShutdown();
        });
        thread.detach();
    }

public:

    virtual bool ReceiveMessage(const MESSAGETYPE *message, const COMPONENT_KEY &sender)
    {
        std::lock_guard<std::mutex> lock(m_MessageBehaviorsMutex);
        bool usedMessage = false;
        for(auto it = m_MessageBehaviors.cbegin() ; it != m_MessageBehaviors.cend() ; ++it)
        {
            bool criteraEvaluation = std::get<0>(*it)(sender, message);
            if(criteraEvaluation)
            {
                std::get<1>(*it)(sender, message);
                usedMessage = true;
            }
        }

        return usedMessage;
    }

public:



    COMPONENT_KEY GetKeyFromSecondaryID(int ID) const
    {
        return m_CB->GetKeyFromSecondaryID(ID);
    }


    COMPONENT_KEY GetHostKey() const
    {
        return m_CB->GetHostKey();
    }

private:

    void onShutdown(){

        m_MessageBehaviorsMutex.lock();
        m_MessageBehaviors.clear();
        m_MessageBehaviorsMutex.unlock();


        std::vector<std::function<void()>> lambdasToCall = {};
        for(auto it = m_ShutdownLambda.cbegin() ; it != m_ShutdownLambda.cend() ; ++it)
        {
            lambdasToCall.push_back(it->second);
        }

        for(auto it = lambdasToCall.cbegin() ; it != lambdasToCall.cend() ; ++it)
        {
            (*it)();
        }
    }

public:


    //!
    //! \brief Queue a transmission to a single target, to be removed by one message type
    //!
    //! This transmission is to be halted when a response of maching key and messagID is observed.
    //!
    //! \template T Unique key type to identify this transmission when combined with messageID
    //! \param key key of entity that is transmitting this message
    //! \param messageID ID of message that is expected to respond to this message
    //! \param transmitAction Action to take to do transmissions
    //!
    template <typename T>
    void QueueTransmission(const T &key, const int &messageID, const COMPONENT_KEY &target, const std::function<void(const std::vector<COMPONENT_KEY> &unheard)> &transmitAction)
    {
        auto lambda = [this](){
            onFinished(false);
        };
        TransmitQueueType::QueueTransmission(ObjectIntTuple<T>(key, messageID), {target}, transmitAction, lambda);
    }


    //!
    //! \brief Queue a transmission to a single target, to be removed by any one of a set of messages
    //!
    //! This transmission is to be halted when a response that matches any of the given messagesID is observed
    //!
    //! \template T Unique key type to identify this transmission when combined with messageID
    //! \param key key of entity that is transmitting this message
    //! \param messageID Vector of messageID's that can remove this transmission
    //! \param transmitAction Action to take to do transmissions
    template <typename T>
    void QueueTransmission(const T &key, const std::vector<int> &messagesID, const COMPONENT_KEY &target, const std::function<void(const std::vector<COMPONENT_KEY> &unheard)> &transmitAction)
    {
        auto lambda = [this](){
            onFinished(false);
        };
        std::vector<ObjectIntTuple<T>> vec;
        for(auto it = messagesID.cbegin() ; it != messagesID.cend() ; ++it)
        {
            vec.push_back(ObjectIntTuple<T>(key, *it));
        }
        TransmitQueueType::QueueTransmission(vec, {target}, transmitAction, lambda);
    }


    //!
    //! \brief Queue a broadcast to a set of targets, each one removed by a single expected message ID
    //!
    //! \template T Unique key type to identify this transmission when combined with messageID
    //! \param key key of entity that is transmitting this message
    //! \param messageID ID of message that is expected to respond to this message
    //! \param transmitAction Action to take to do transmissions
    //!
    template <typename T>
    void QueueReliableBroadcast(const T &key, const int &messageID, const std::vector<COMPONENT_KEY> &targets, const std::function<void(const std::vector<COMPONENT_KEY> &unheard)> &transmitAction)
    {
        auto lambda = [this](){
            onFinished(false);
        };

        TransmitQueueType::QueueTransmission(ObjectIntTuple<T>(key, messageID), targets, transmitAction, lambda);
    }


    //!
    //! \brief Remove transmission from Queue.
    //!
    //! If a match queued transmissions that maches BOTH given key and messageID is found, then the tranmission is removed.
    //!
    //! \template T Key type of transmission to remove
    //! \param key Object to check for equivlance
    //! \param messageID ID of message received to check for
    //! \param receivedFrom Component that message was received from, needed to keep track of tranmissions to multiple targets (reliable broadcasts)
    //!
    template <typename T>
    void RemoveTransmission(const T &key, const int &messageID, OptionalParameter<COMPONENT_KEY> receivedFrom = OptionalParameter<COMPONENT_KEY>())
    {
        TransmitQueueType::RemoveTransmission(ObjectIntTuple<T>(key, messageID), receivedFrom);
    }

    /*!
     * \brief Removes all queued transmissions.
     */
    void RemoveAllTransmissions()
    {
        TransmitQueueType::RemoveAllTransmissions();
    }


    //!
    //! \brief Add logic that is triggered by unsolicited message
    //!
    //! The logic added by this function does NOT delete any queued transmission.
    //!
    //! \template I Message id this logic pertains to
    //! \template DECODE_TYPE type that received message is decoded to
    //! \template DECODE_FUNC function to decode received message to DECODE_TYPE
    //! \param func function to decode the message received.
    //! \param action Action to partake with message.
    //!
    template <const int I, typename DECODE_TYPE, typename DECODE_FUNC>
    void AddTriggeredLogic(DECODE_FUNC func, const std::function<void(const DECODE_TYPE &msg, const COMPONENT_KEY &sender)> &action)
    {

        auto newItem = std::make_tuple(MaceMessageIDEq<I>(),
                                       MaceProcessFSMState<DECODE_TYPE>(func, [this, action](const DECODE_TYPE &msg, const COMPONENT_KEY &sender)
                                        {
                                            action(msg, sender);
                                        }));

        std::lock_guard<std::mutex> lock(m_MessageBehaviorsMutex);
        m_MessageBehaviors.push_back(newItem);
    }


public:


    template <typename FUNC, typename TT>
    void EncodeMessage(FUNC func, TT requestItem, const COMPONENT_KEY &sender, const OptionalParameter<COMPONENT_KEY> &target = OptionalParameter<COMPONENT_KEY>())
    {
        MESSAGETYPE msg;
        func(sender, m_LinkChan, &msg, &requestItem);
        m_CB->TransmitMessage(msg, target);
    }


private:

    template <const int I>
    static std::function<bool(COMPONENT_KEY, const MESSAGETYPE*)> MaceMessageIDEq()
    {
        return [](COMPONENT_KEY sender, const MESSAGETYPE* message){
            UNUSED(sender);
            return message->msgid ==I;
        };
    }

    template <typename DECODE_TYPE, typename DECODE_FUNC>
    static std::function<void(COMPONENT_KEY, const MESSAGETYPE*)> MaceProcessFSMState(DECODE_FUNC decode_func, const std::function<void(const DECODE_TYPE &msg, const COMPONENT_KEY &sender)> &func)
    {
        return [decode_func, func](COMPONENT_KEY sender, const MESSAGETYPE* message)
        {
            DECODE_TYPE decodedMSG;
            decode_func(message, &decodedMSG);

            func(decodedMSG, sender);
        };
    }

    GenericController(const GenericController &rhs);
    GenericController& operator=(const GenericController&);

};

}


#endif // GENERIC_MACE_CONTROLLER_H
