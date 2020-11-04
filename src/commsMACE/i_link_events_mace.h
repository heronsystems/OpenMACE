#ifndef I_LINK_EVENTS_MACE_H
#define I_LINK_EVENTS_MACE_H

#include "commsmace_global.h"

#include <cstdlib>
#include <vector>
#include <string>

#ifndef RESOURCE_H
#define RESOURCE_H

#include <cstring>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <algorithm>

#include "common/common.h"

class ResourceKey : public std::vector<std::string>
{

public:

    ResourceKey()
    {

    }

    ResourceKey(std::string name) :
        std::vector<std::string>({name})
    {

    }

    ResourceKey(const std::vector<std::string> &vec) :
        std::vector<std::string>(vec)
    {

    }

    //!
    //! \brief Determine if the given key is a sub-unit of this key.
    //!
    //! For example if this key is [A B] and the given key is [A] then this function would return true
    //! \param rhs Given key
    //! \param subMatch
    //! \return True if given key is part of this key
    //!
    bool containsKey(const ResourceKey &rhs) const
    {
        for(size_t i = 0 ; i < rhs.size() ; i++)
        {
            std::string nameInGivenKey = rhs.at(i);

            bool found = false;
            for(size_t j = 0 ; j < this->size() ; j++)
            {
                std::string nameInThis = this->at(j);
                if(nameInThis == nameInGivenKey)
                {
                    found = true;
                    break;
                }
            }
            if(found == false)
            {
                return false;
            }
        }

        return true;
    }

    void AddNameToResourceKey(const std::string &name)
    {
        push_back(name);
    }

    std::size_t hash() const {
      std::size_t seed = size();
      for(auto& str : *this) {
        std::size_t h0 = 0;
        for(std::size_t i = 0 ; i < str.size() ; i++)
        {
            h0 += static_cast<size_t>(str[i]);
        }
        seed ^= h0 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }

};


namespace std {

  template <>
  struct hash<ResourceKey>
  {
    std::size_t operator()(const ResourceKey& k) const
    {
      return k.hash();
    }
  };

}





class ResourceValue : public std::vector<int>
{
private:


public:

    ResourceValue()
    {

    }

    ResourceValue(int name) :
        std::vector<int>({name})
    {

    }

    ResourceValue(const std::vector<int> &vec) :
        std::vector<int>(vec)
    {

    }

    void AddValueToResourceKey(const int name)
    {
        push_back(name);
    }

    std::size_t hash() const {
      std::size_t seed = size();
      for(auto& i : *this) {
        seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
};


namespace std {

  template <>
  struct hash<ResourceValue>
  {
    std::size_t operator()(const ResourceValue& k) const
    {
      return k.hash();
    }
  };

}


class ResourceList
{
private:
    std::unordered_map<ResourceKey, std::vector<ResourceValue>> m_InternalResources;
    std::unordered_map<ResourceKey, std::unordered_map<ResourceValue, uint64_t>> m_ResourcesToRadioAddr;

    std::mutex m_Mutex;

public:

    void AddInternalResource(const ResourceKey &key, const ResourceValue &value)
    {
        if(m_ResourcesToRadioAddr.find(key) == m_ResourcesToRadioAddr.cend())
        {
            m_ResourcesToRadioAddr.insert({key, {}});
            m_InternalResources.insert({key, {}});
        }

        if(m_ResourcesToRadioAddr.at(key).find(value) != m_ResourcesToRadioAddr.at(key).cend()){
            throw std::runtime_error("Resource of given ID already exists");
        }

        m_Mutex.lock();
        m_ResourcesToRadioAddr.at(key).insert({value, 0x00});
        m_Mutex.unlock();

        m_InternalResources.at(key).push_back(value);
    }

    void RemoveInternalResource(const ResourceKey &key, const ResourceValue &value)
    {
        if(m_ResourcesToRadioAddr.find(key) == m_ResourcesToRadioAddr.cend())
        {
            return;
        }

        if(m_ResourcesToRadioAddr.at(key).find(value) == m_ResourcesToRadioAddr.at(key).cend()){
            return;
        }

        m_Mutex.lock();
        m_ResourcesToRadioAddr.at(key).erase(m_ResourcesToRadioAddr.at(key).find(value));
        m_Mutex.unlock();

        for(auto it = m_InternalResources.at(key).cbegin() ; it != m_InternalResources.at(key).cend() ; ++it)
        {
            if(*it == value)
            {
                m_InternalResources.at(key).erase(it);
                break;
            }
        }
    }

    //!
    //! \brief Given a new resource value and address, add it to the known external resources
    //! \param key
    //! \param value
    //! \param addr
    //! \return True if added, false if already added
    //!
    bool AddExternalResource(const ResourceKey &key, const ResourceValue &value, uint64_t addr)
    {
        if(m_ResourcesToRadioAddr.find(key) == m_ResourcesToRadioAddr.cend())
        {
            m_ResourcesToRadioAddr.insert({key, {}});
            m_InternalResources.insert({key, {}});
        }

        if(m_ResourcesToRadioAddr.at(key).find(value) != m_ResourcesToRadioAddr.at(key).cend()){

            //check if the addr is different (and not set to self)
            if(m_ResourcesToRadioAddr.at(key).at(value) != 0 && m_ResourcesToRadioAddr.at(key).at(value) != addr)
            {
                throw std::runtime_error("Given Resource has already been added with a different address");
            }
            return false;
        }

        m_Mutex.lock();
        m_ResourcesToRadioAddr.at(key).insert({value, addr});
        m_Mutex.unlock();

        return true;
    }

    void RemoveExternalResource(const ResourceKey &key, const ResourceValue &value)
    {
        if(m_ResourcesToRadioAddr.find(key) == m_ResourcesToRadioAddr.cend())
        {
            return;
        }

        if(m_ResourcesToRadioAddr.at(key).find(value) == m_ResourcesToRadioAddr.at(key).cend()){
            return;
        }

        m_Mutex.lock();
        m_ResourcesToRadioAddr.at(key).erase(m_ResourcesToRadioAddr.at(key).find(value));
        m_Mutex.unlock();
    }

    bool HasAddr(const ResourceKey &key, const ResourceValue &value) const
    {
        if(m_ResourcesToRadioAddr.find(key) == m_ResourcesToRadioAddr.cend())
        {
            return false;
        }

        if(m_ResourcesToRadioAddr.at(key).find(value) == m_ResourcesToRadioAddr.at(key).cend()){
            return false;
        }

        return true;
    }

    uint64_t GetAddr(const ResourceKey &key, const ResourceValue &value)
    {
        return m_ResourcesToRadioAddr.at(key).at(value);
    }

    std::vector<std::tuple<ResourceKey, ResourceValue>> getResourcesMatch(const ResourceKey &key, bool subMatch = true, bool internalOnly = false)
    {
        UNUSED(subMatch);
        std::vector<std::tuple<ResourceKey, ResourceValue>> rtn;

        m_Mutex.lock();
        for(auto it = m_ResourcesToRadioAddr.cbegin() ; it != m_ResourcesToRadioAddr.cend() ; ++it)
        {
            ResourceKey keyToCheck = it->first;
            if(keyToCheck.containsKey(key))
            {
                for(auto itt = it->second.cbegin() ; itt != it->second.cend() ; ++itt)
                {
                    /// if only interested in internal then skip any non internal
                    if(internalOnly == true && itt->second != 0)
                    {
                        continue;
                    }
                    rtn.push_back(std::make_tuple(it->first, itt->first));
                }
            }
        }
        m_Mutex.unlock();

        return rtn;
    }

public:
};


#endif // RESOURCE_H


namespace CommsMACE
{

//!
//! \brief Describes a targetable resource, either on local machine or remote
//!
//! A resource is defined as a list of components and assosiated list of ID's
//! Examples:
//!   [MaceInstance(1)]
//!   [MaceInstance(1) Vehicle(1)]
//!   [MaceInstance(1) Vehicle(2)]
//!
class Resource
{
private:
    std::vector<std::string> m_componentNames;
    std::vector<int> m_IDs;
public:

    void Add(const std::string &name, const int ID)
    {
        m_componentNames.push_back(name);
        m_IDs.push_back(ID);
    }


    template<char* ...N, typename ...I>
    void Set(I... ids)
    {
        static_assert(sizeof...(N) == sizeof...(ids), "Name and Resource values length must be the same");


        m_componentNames =  { N... };
        m_IDs = { (int)ids... }; // TODO: Is casting here appropriate? It was for suppressing "-wnarrowing" warnings
    }

    size_t Size() const
    {
        return m_IDs.size();
    }

    std::string NameAt(unsigned int i) const
    {
        return m_componentNames.at(i);
    }

    int IDAt(unsigned int i) const
    {
        return m_IDs.at(i);
    }

    bool operator ==(const Resource &rhs) const
    {
        if(this->m_IDs.size() != rhs.m_componentNames.size()) return false;
        if(this->m_componentNames.size() != rhs.m_componentNames.size()) return false;

        for(std::size_t i = 0 ; i < m_IDs.size() ; i++)
        {
            if(m_IDs.at(i) != rhs.m_IDs.at(i)) return false;
        }

        for(std::size_t i = 0 ; i < m_componentNames.size() ; i++)
        {
            if(m_componentNames.at(i) != rhs.m_componentNames.at(i)) return false;
        }

        return true;
    }

};







class ILink;

class ILinkEvents
{
public:
    virtual ~ILinkEvents() = default;
public:

    virtual void AddedExternalResource(ILink *link_ptr, const Resource &resource) = 0;

    virtual void RemovedExternalResource(ILink *link_ptr, const Resource &resource) const = 0;

    virtual void ReceiveData(ILink *link_ptr, const std::vector<uint8_t> &buffer) const = 0;

    virtual void CommunicationError(const ILink* link_ptr, const std::string &type, const std::string &msg) const = 0;

    virtual void CommunicationUpdate(const ILink *link_ptr, const std::string &name, const std::string &msg) const = 0;

    virtual void Connected(const ILink* link_ptr) const = 0;

    virtual void ConnectionRemoved(const ILink *link_ptr) const = 0;
};

} //END Comms



namespace std
{

template<>
struct hash<CommsMACE::Resource>
{
    std::size_t operator()(const CommsMACE::Resource& k) const
    {



        using std::size_t;
        using std::hash;
        using std::string;

        std::size_t value = 0;

        for(size_t i = 0 ; i < k.Size() ; i++)
        {
            value ^= std::hash<std::string>()(k.NameAt(i));
            value ^= std::hash<int>()(k.IDAt(i));
        }

        return value;
    }
};
}

#endif // I_LINK_EVENTS_H
