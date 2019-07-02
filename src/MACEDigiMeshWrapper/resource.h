#ifndef RESOURCE_H
#define RESOURCE_H

#include <cstring>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <algorithm>

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
        for(int i = 0 ; i < rhs.size() ; i++)
        {
            std::string nameInGivenKey = rhs.at(i);

            bool found = false;
            for(int j = 0 ; j < this->size() ; j++)
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
            h0 += str[i];
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


/*
class SingleResource
{
private:

    ResourceKey m_Key;
    ResourceValue m_Value;

public:

    SingleResource(ResourceKey &key, ResourceValue &value) :
        m_Key(key),
        m_Value(value)
    {

    }

    void AddIdentifiedComponent(const ResourceKey &key, const ResourceValue &value)
    {
        m_Key = key;
        m_Value = value;
    }

    std::size_t Size()
    {
        return m_Key.size();
    }

    const char* KeyAt(size_t i) const
    {
        return m_Key.at(i);
    }

    int ValueAt(size_t i) const
    {
        return m_Value.at(i);
    }
};
*/



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
