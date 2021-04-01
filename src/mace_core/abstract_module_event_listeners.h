#ifndef MODULE_ABSTRACT_EVENT_LISTENERS_H
#define MODULE_ABSTRACT_EVENT_LISTENERS_H

#include "abstract_module_base.h"

#include "command_marshler.h"

#include "i_module_topic_events.h"
#include <algorithm>
#include "module_characteristics.h"

namespace MaceCore
{

//!
//! \brief Build up of ModuleBase that adds event listening helper functions
//!
template<typename T, typename I, typename CT>
class AbstractModule_EventListeners : public ModuleBase
{
public:

    AbstractModule_EventListeners() :
        m_LoopSleepTime(std::chrono::milliseconds(10)),
        m_DefaultMarshalCommandOnEventLoop(true),
        m_Started(false),
        m_Shutdown(false)
    {

    }



    virtual void start()
    {
        m_Started = true;
        while(true)
        {
            if(m_Shutdown == true)
            {
                break;
            }

            m_CommandDispatcher.ExecuteQueuedCommands();

            std::this_thread::sleep_for (m_LoopSleepTime);
        }
    }

    virtual void shutdown()
    {
        m_Shutdown = true;
    }

    void setModuleMetaData(const T &data)
    {
        m_MetaData = data;
    }

    T getModuleMetaData()
    {
        return m_MetaData;
    }

    bool IsStarted() const  {
        return m_Started;
    }



    //!
    //! \brief Add a listener to the module
    //! Multiple listners can be added if desired
    //! \param listener Listener to invoke module events onto
    //!
    void addListener(I *listener)
    {
        m_Listeners.push_back(listener);
    }

    void addTopicListener(IModuleTopicEvents *listener)
    {
        std::lock_guard<std::mutex> lock(m_TopicListenerMutex);
        m_TopicListeners.push_back(listener);
        AttachedAsModule(listener);
    }

    void RemoveTopicListener(IModuleTopicEvents *listener) {

        std::lock_guard<std::mutex> lock(m_TopicListenerMutex);

        m_TopicListeners.erase(std::remove_if(m_TopicListeners.begin(),
                                              m_TopicListeners.end(),
                                              [listener](IModuleTopicEvents * x){ return x == listener;}),
                               m_TopicListeners.end());
    }

    void NotifyListenersOfTopic(const std::function<void(IModuleTopicEvents*)> &func) const
    {
        for(auto it = m_TopicListeners.cbegin() ; it != m_TopicListeners.cend() ; ++it)
        {
            func(*it);
        }
    }


    //!
    //! \brief Notify all listeners of some event
    //!
    //! The event itself is described in the provided lambda function
    //! This function exists purley to simplify the need to loop over all listners
    //! \param func Lambda function that invokes event on some object
    //!
    void NotifyListeners(const std::function<void(I*)> &func) const
    {
        for(auto it = m_Listeners.cbegin() ; it != m_Listeners.cend() ; ++it)
        {
            func(*it);
        }
    }


    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(IModuleTopicEvents* ptr) = 0;


    //////////////////////////////////////////////////////////////////////
    ///     COMMAND MANAGMENT
    //////////////////////////////////////////////////////////////////////

public:

    //!
    //! \brief Add zero-parameter logic (in form of lambda function) to a command
    //! \param command Command to add logic to
    //! \param logic Lambda function to call when command is invoked.
    //!
    void AddCommandLogic(const CT command, const std::function<void(const OptionalParameter<ModuleCharacteristic> &sender)> &logic)
    {
        m_CommandDispatcher.AddLambda(command, logic);
    }


    //!
    //! \brief Add one-parameter logic (in form of lambda function) to a command
    //! \param command Command to add logic to
    //! \param logic Lambda function to call when command is invoked.
    //!
    template<typename P1T>
    void AddCommandLogic(const CT command, const std::function<void(const P1T&, const OptionalParameter<ModuleCharacteristic> &sender)> &lambda)
    {
        m_CommandDispatcher.AddLambda(command, lambda);
    }


    //!
    //! \brief Add two-parameter logic (in form of lambda function) to a command
    //! \param command Command to add logic to
    //! \param logic Lambda function to call when command is invoked.
    //!
    template<typename P1T, typename P2T>
    void AddCommandLogic(const CT command, const std::function<void(const P1T&, const P2T&, const OptionalParameter<ModuleCharacteristic> &sender)> &lambda)
    {
        m_CommandDispatcher.AddLambda(command, lambda);
    }


    //!
    //! \brief Set the maximum rate which commands are to be called
    //!
    //! The maximum rate which all commands can be invoked, regardless of SetMaxRate_SpecificCommand settings.
    //! It is not recomended this value is set to 0 to prevent unessessary spinning.
    //!
    //! Used when commands are being marshaled on the built-in event loop
    //! Setting this will restrict the command's to only be invoked at a rate given
    //! So if the command is called twice in row, the built-in event loop will wait the provided amount before calling again.
    //!
    //! \param duration Interval in ms which built-in event loop is to wait prior to checking.
    //!
    void SetMaxRate(const std::chrono::milliseconds &duration)
    {
        m_LoopSleepTime = duration;
    }


    //!
    //! \brief Set maximum rate which a command can be called.
    //!
    //! If interval set to smaller value than SetMaxRate, the larger interval will be effective.
    //!
    //! Used when the command is being marshaled on the built-in event loop.
    //! Setting this will restrict the command's to only be invoked at a rate given
    //! So if the command is called twice in row, the built-in event loop will wait the provided amount before calling again.
    //! \param command Command to set interval of
    //! \param interval Interval in milliseconds between call
    //!
    void SetMaxRate_SpecificCommand(CT command, std::chrono::milliseconds interval)
    {
        m_CommandDispatcher.SetCallInterval(command, interval);
    }


    //!
    //! \brief Define default behavior as to commands are to invoked on event loop
    //!
    //! If set to true, the execution of a commands will occur on the module's thread at a rate that doesn't exceed the module's/command's rates.
    //! Code placed into module's command methods will be executed on module's thread.
    //!
    //! If set to false, the executation of a command will occur on the thread that decided the command must be called at the moment which it was decided.
    //! It will then be up to the implementor to marshal that command onto module's thread by overriding start() method.
    //! \param value True if are to marshal onto event loop
    //!
    void InvokeCommandOnModuleEventLoop(const bool &value)
    {
        m_DefaultMarshalCommandOnEventLoop = value;
    }


    //!
    //! \brief Define for a specific command if it should be invoked on event loop
    //!
    //! If set to true, the execution of a commands will occur on the module's thread at a rate that doesn't exceed the module's/command's rates.
    //! Code placed into module's command methods will be executed on module's thread.
    //!
    //! If set to false, the executation of a command will occur on the thread that decided the command must be called at the moment which it was decided.
    //! It will then be up to the implementor to marshal that command onto module's thread by overriding start() method.
    //!
    //! If some commands are to be invoked on module's event loop and other not, the start() method will need to be overidden and a custom event loop written
    //! In that event loop, any commands who are to use built-in marshalling can be executed with "ExecuteQueuedCommands();" command.
    //!
    //! \param command Command to modify behavior of.
    //! \param value True if are to marshal onto event loop
    //!
    void InvokeCommandOnModuleEventLoop(const CT command, const bool &value)
    {
        m_MarshalCommandsOnEventLoop[command] = value;
    }

protected:


    //!
    //! \brief Execute any queued asyncrous commands on the thread which this method was called on
    //!
    void ExecuteQueuedCommands()
    {
        m_CommandDispatcher.ExecuteQueuedCommands();
    }


    //!
    //! \brief Execute a module's command with no parameters
    //! \param enumValue Command to call.
    //!
    void MarshalCommand(CT enumValue, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>())
    {
        bool IssueOnThread;
        if(m_MarshalCommandsOnEventLoop.find(enumValue) == m_MarshalCommandsOnEventLoop.cend())
            IssueOnThread = m_DefaultMarshalCommandOnEventLoop;
        else
            IssueOnThread = m_MarshalCommandsOnEventLoop.at(enumValue);

        if(IssueOnThread == true)
            this->m_CommandDispatcher.QueueCommand(enumValue, sender);
        else
            this->m_CommandDispatcher.ImmediatlyCallCommand(enumValue, sender);
    }


    //!
    //! \brief Execute a module's command with one parameters
    //! \param enumValue Command to call.
    //! \param value Value of first parameter
    //!
    template<typename P1T>
    void MarshalCommand(CT enumValue, const P1T &value, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>())
    {
        bool IssueOnModuleEventLoop;
        if(m_MarshalCommandsOnEventLoop.find(enumValue) == m_MarshalCommandsOnEventLoop.cend())
            IssueOnModuleEventLoop = m_DefaultMarshalCommandOnEventLoop;
        else
            IssueOnModuleEventLoop = m_MarshalCommandsOnEventLoop.at(enumValue);
        if(IssueOnModuleEventLoop == true)

            this->m_CommandDispatcher.QueueCommand(enumValue, value, sender);
        else
            this->m_CommandDispatcher.ImmediatlyCallCommand(enumValue, value, sender);
    }

    //!
    //! \brief Execute a module's command with one parameters
    //! \param enumValue Command to call.
    //! \param value Value of first parameter
    //!
    template<typename P1T, typename P2T>
    void MarshalCommandTwoParameter(CT enumValue, const P1T &value1, const P2T &value2, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>())
    {
        bool IssueOnModuleEventLoop;
        if(m_MarshalCommandsOnEventLoop.find(enumValue) == m_MarshalCommandsOnEventLoop.cend())
            IssueOnModuleEventLoop = m_DefaultMarshalCommandOnEventLoop;
        else
            IssueOnModuleEventLoop = m_MarshalCommandsOnEventLoop.at(enumValue);

        if(IssueOnModuleEventLoop == true)
            this->m_CommandDispatcher.QueueCommand(enumValue, value1, value2, sender);
        else
            this->m_CommandDispatcher.ImmediatlyCallCommand(enumValue, value1, value2, sender);
    }



private:

    T m_MetaData;

    std::vector<I*> m_Listeners;
    std::mutex m_TopicListenerMutex;
    std::vector<IModuleTopicEvents*> m_TopicListeners;

    std::chrono::milliseconds m_LoopSleepTime;


    bool m_DefaultMarshalCommandOnEventLoop;


    std::unordered_map<CT, bool, EnumClassHash> m_MarshalCommandsOnEventLoop;


    CommandMarshler<CT> m_CommandDispatcher;

    bool m_Started;
    bool m_Shutdown;

};

} // END MaceCore Namespace

#endif // MODULE_ABSTRACT_EVENT_LISTENERS_H
