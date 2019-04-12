#ifndef EVENT_LOOP_MARSHLER_H
#define EVENT_LOOP_MARSHLER_H

#include <functional>
#include <unordered_map>
#include <vector>
#include <list>
#include <mutex>
#include <memory>
#include <ctime>

#include "mace_core_global.h"

#include "module_characteristics.h"


//!
//! \brief Object that stores functions to call given some enumeration.
//!
//! Can either store calls to be invoked at a later time (potentially on another thread)
//! Or provide a way to immediatly call the function.
//!
template<typename T>
class CommandMarshler
{
private:


    class FunctionCallerBase
    {
    public:
        virtual void Call() = 0;

        virtual int NumCalls() const = 0;

        void AddSender(const MaceCore::ModuleCharacteristic &sender)
        {
           m_Sender = sender;
        }

    protected:

        std::mutex m_Mutex;

        OptionalParameter<MaceCore::ModuleCharacteristic> m_Sender;
    };


    class ZeroParamCaller : public FunctionCallerBase
    {
    public:
        ZeroParamCaller(const std::function<void(const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &func) :
            m_Func(func)
        {
        }

        void Enable()
        {
            isEnabled = true;
        }

        virtual int NumCalls() const
        {
            if(isEnabled)
                return 1;
            return 0;
        }

        virtual void Call()
        {
            isEnabled = false;
            m_Func(this->m_Sender);
        }

        void InvokeOneInstance(const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
        {
            m_Func(sender);
        }

    private:

        const std::function<void(const OptionalParameter<MaceCore::ModuleCharacteristic> &)> m_Func;

        bool isEnabled;
    };


    template <typename FT>
    class OneParamCaller : public FunctionCallerBase
    {
    public:
        OneParamCaller(const std::function<void(const FT&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &func) :
            m_Func(func)
        {

        }

        void AddParameter(const FT& param)
        {
            this->m_Mutex.lock();
            m_Param1.push_back(param);
            m_Param1.unique();
            this->m_Mutex.unlock();
        }

        virtual int NumCalls() const
        {
            return m_Param1.size();
        }

        virtual void Call()
        {
            this->m_Mutex.lock();
            std::list<FT> listCpy = m_Param1;
            m_Param1.clear();
            this->m_Mutex.unlock();

            for(FT param : listCpy)
                m_Func(param, this->m_Sender);
        }

        void InvokeOneInstance(const FT& param, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
        {
            m_Func(param, sender);
        }

    private:

        const std::function<void(const FT&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> m_Func;

        std::list<FT> m_Param1;
    };



    template <typename FT1, typename FT2>
    class TwoParamCaller : public FunctionCallerBase
    {
    public:
        TwoParamCaller(const std::function<void(const FT1&, const FT2&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &func) :
            m_Func(func)
        {

        }

        void AddParameter(const FT1& param1, const FT2& param2)
        {
            this->m_Mutex.lock();
            m_Param1.push_back(std::make_tuple(param1, param2));
            m_Param1.unique();
            this->m_Mutex.unlock();
        }

        virtual int NumCalls() const
        {
            return m_Param1.size();
        }

        virtual void Call()
        {
            this->m_Mutex.lock();
            std::list<std::tuple<FT1, FT2>> listCpy = m_Param1;
            m_Param1.clear();
            this->m_Mutex.unlock();

            for(std::tuple<FT1, FT2> param : listCpy)
                m_Func(std::get<0>(param), std::get<1>(param), this->m_Sender);
        }

        void InvokeOneInstance(const FT1& param1, const FT2& param2, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
        {
            m_Func(param1, param2, sender);
        }

    private:

        const std::function<void(const FT1&, const FT2&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> m_Func;

        std::list<std::tuple<FT1, FT2>> m_Param1;
    };


public:


    //!
    //! \brief Add a parameterless function that gets fired when given command is issued
    //! \param event Command to triger lambda on
    //! \param lambda Lambda to trigger
    //!
    void AddLambda(T event, const std::function<void(const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &lambda)
    {
        m_CallRate.insert({event, std::chrono::milliseconds(0)});
        m_LastCallTime.insert({event, 0});
        m_EventProcedures.insert({event, std::make_shared<ZeroParamCaller>(lambda)});
    }


    //!
    //! \brief Add a 1 parameter function that gets fired when given command is issued
    //! \param event Command to triger lambda on
    //! \param lambda Lambda to trigger
    //!
    template<typename P1T>
    void AddLambda(T event, const std::function<void(const P1T&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &lambda)
    {
        m_CallRate.insert({event, std::chrono::milliseconds(0)});
        m_LastCallTime.insert({event, 0});
        m_EventProcedures.insert({event, std::make_shared<OneParamCaller<P1T>>(lambda)});
    }


    //!
    //! \brief Add a 2 parameter function that gets fired when given command is issued
    //! \param event Command to triger lambda on
    //! \param lambda Lambda to trigger
    //!
    template<typename P1T, typename P2T>
    void AddLambda(T event, const std::function<void(const P1T&, const P2T&, const OptionalParameter<MaceCore::ModuleCharacteristic>&)> &lambda)
    {
        m_CallRate.insert({event, std::chrono::milliseconds(0)});
        m_LastCallTime.insert({event, 0});
        m_EventProcedures.insert({event, std::make_shared<TwoParamCaller<P1T, P2T>>(lambda)});
    }


    //!
    //! \brief Queue the fireing of a parameterless command
    //! \param event Command to queue the calling of internal lambda
    //!
    void QueueCommand(T event, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        ZeroParamCaller *caller = (ZeroParamCaller*)m_EventProcedures.at(event).get();
        if(sender.IsSet() == true)
        {
            caller->AddSender(sender.Value());
        }
        caller->Enable();
    }


    //!
    //! \brief Queue the fireing of a single-parameter command
    //! \param event Command to queue the calling of internal lambda
    //!
    template<typename P1T>
    void QueueCommand(T event, const P1T &param1, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        OneParamCaller<P1T> *caller = (OneParamCaller<P1T>*)m_EventProcedures.at(event).get();
        caller->AddParameter(param1);
        if(sender.IsSet() == true)
        {
            caller->AddSender(sender.Value());
        }
    }


    //!
    //! \brief Queue the fireing of a two-parameter command
    //! \param event Command to queue the calling of internal lambda
    //!
    template<typename P1T,typename P2T>
    void QueueCommand(T event, const P1T &param1, const P2T &param2, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        TwoParamCaller<P1T, P2T> *caller = (TwoParamCaller<P1T, P2T>*)m_EventProcedures.at(event).get();
        caller->AddParameter(param1, param2);
        if(sender.IsSet() == true)
        {
            caller->AddSender(sender.Value());
        }
    }


    //!
    //! \brief Immediatly invoke the command on the thread this method is called from
    //! \param event Command to immediatly invoke
    //!
    void ImmediatlyCallCommand(T event, OptionalParameter<MaceCore::ModuleCharacteristic> sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        ZeroParamCaller *caller = (ZeroParamCaller*)m_EventProcedures.at(event).get();
        caller->InvokeOneInstance(sender);
    }


    //!
    //! \brief Immediatly invoke the command on the thread this method is called from
    //! \param event Command to immediatly invoke
    //!
    template<typename P1T>
    void ImmediatlyCallCommand(T event, const P1T &param1, OptionalParameter<MaceCore::ModuleCharacteristic> sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        OneParamCaller<P1T> *caller = (OneParamCaller<P1T>*)m_EventProcedures.at(event).get();
        caller->InvokeOneInstance(param1, sender);
    }


    //!
    //! \brief Immediatly invoke the command on the thread this method is called from
    //! \param event Command to immediatly invoke
    //!
    template<typename P1T, typename P2T>
    void ImmediatlyCallCommand(T event, const P1T &param1, const P2T &param2, OptionalParameter<MaceCore::ModuleCharacteristic> sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {
        if(m_EventProcedures.find(event) == m_EventProcedures.cend())
            throw std::runtime_error("Given command does not have a behavior defined");

        TwoParamCaller<P1T, P2T> *caller = (TwoParamCaller<P1T, P2T>*)m_EventProcedures.at(event).get();
        caller->InvokeOneInstance(param1, param2, sender);
    }


    //!
    //! \brief Execute lambdas for all queued commands
    //!
    void ExecuteQueuedCommands()
    {
        //make a list of events that are to be calls.
        std::vector<T> list;
        for(auto it = m_EventProcedures.cbegin() ; it != m_EventProcedures.cend() ; ++it)
        {
            T enumValue = it->first;

            std::clock_t lastTime = m_LastCallTime.at(enumValue);
            std::clock_t currTime = std::clock();

            double msDiff = (currTime - lastTime) / (double)(CLOCKS_PER_SEC / 1000.0);
            std::chrono::milliseconds rate = m_CallRate.at(enumValue);
            if(msDiff >= rate.count())
            {

                std::shared_ptr<FunctionCallerBase> caller = it->second;

                if(caller->NumCalls() > 0)
                    caller->Call();

                m_LastCallTime[enumValue] = currTime;
            }
        }
    }


    //!
    //! \brief Set the call interval of a command
    //! \param event Command to set interval of
    //! \param interval Interval to set to
    //!
    void SetCallInterval(T event, const std::chrono::milliseconds interval)
    {
        m_CallRate[event] = interval;
    }


private:



    std::unordered_map<T, std::shared_ptr<FunctionCallerBase>, EnumClassHash> m_EventProcedures;

    std::unordered_map<T, std::chrono::milliseconds, EnumClassHash> m_CallRate;

    std::unordered_map<T, std::clock_t, EnumClassHash> m_LastCallTime;
};

#endif // EVENT_LOOP_MARSHLER_H
