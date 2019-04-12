#ifndef DATA_NOTIFIER_H
#define DATA_NOTIFIER_H

#include <functional>
#include <algorithm>
#include <mutex>
#include <unordered_map>
namespace Data
{

template <typename T>
class DataGetSetNotifier
{
public:
    DataGetSetNotifier()  :
        m_BeenSet(false)
    {

    }

    void AddNotifier(void* obj, const std::function<void()> func) {
        std::lock_guard<std::mutex> guardData(m_NotifierListMutex);

        //m_NotifierListMutex.lock();
        //std::lock lock(m_NotifierListMutex);

        if(m_Funcs.find(obj) == m_Funcs.cend()) {
            m_Funcs.insert({obj, func});
        }
        else {
            m_Funcs[obj] = func;
        }

        //m_NotifierListMutex.unlock();
    }

    void RemoveNotifier(void* obj) {
        //std::lock lock(m_NotifierListMutex);
        std::lock_guard<std::mutex> guardNotifier(m_NotifierListMutex);

        if(m_Funcs.find(obj) != m_Funcs.cend()) {
            m_Funcs.erase(obj);
        }
    }

    bool hasBeenSet() const
    {
        return m_BeenSet;
    }

    bool set(const T &data) {
        //std::lock_guard<std::mutex> guardData(m_AccessMutex);
        m_AccessMutex.lock();
        if(m_BeenSet == true && m_Data == data) {
            m_AccessMutex.unlock();
            return false;
        }

        m_Data = data;
        m_BeenSet = true;
        m_AccessMutex.unlock();


        std::lock_guard<std::mutex> guardNotifier(m_NotifierListMutex);
        for(auto it = m_Funcs.cbegin() ; it != m_Funcs.cend() ; ++it) {
            std::function<void()> func = it->second;
            func();
        }
        return true;
    }

    T get() const {
        std::lock_guard<std::mutex> guardData(m_AccessMutex);
        //std::lock lock(m_AccessMutex);
        return m_Data;
    }

    void operator = (const T &rhs) {
        this->set(rhs);
    }

private:
    std::unordered_map<void*, std::function<void()>> m_Funcs;
    T m_Data;
    bool m_BeenSet;
    mutable std::mutex m_AccessMutex;
    mutable std::mutex m_NotifierListMutex;
};

}

#endif // DATA_NOTIFIER_H
