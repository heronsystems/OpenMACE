#ifndef TOPIC_COMPONENT_COLLECTION_H
#define TOPIC_COMPONENT_COLLECTION_H
#include <functional>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include "common/common.h"
#include "i_topic_component_data_object.h"

namespace Data {

template <typename... Args>
class TopicDataObjectCollection;

template <>
class TopicDataObjectCollection<>
{
public:
    TopicDataObjectCollection(const std::string &name) :
        m_TopicName(name)
    {

    }

//TODO: Kenny commented this out as it was unused
//    MaceCore::TopicDatagram GenerateDatagram(const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &list) const
//    {
//        UNUSED(list);
//    }


    static void SetComponent(std::shared_ptr<ITopicComponentDataObject> ptr, MaceCore::TopicDatagram &datagram) {
        //TODO: WOULD PREFER THIS TO BE THROWN ON COMPILE INSTEAD OF RUNTIME
        UNUSED(ptr);
        UNUSED(datagram);
        throw std::runtime_error("Unknown component passed to topic");
    }


    static bool GetComponent(std::shared_ptr<ITopicComponentDataObject> ptr, const MaceCore::TopicDatagram &datagram) {
        //TODO: WOULD PREFER THIS TO BE THROWN ON COMPILE INSTEAD OF RUNTIME
        UNUSED(ptr);
        UNUSED(datagram);
        throw std::runtime_error("Unknown component passed to topic");
    }


    std::string Name() {
        return m_TopicName;
    }



protected:
    std::string m_TopicName;
};



template <typename T, typename... Args>
class TopicDataObjectCollection<T, Args...> : public TopicDataObjectCollection<Args...>
{
public:
    //using TopicDataObjectCollection<Args...>::SetComponent;
    //using TopicDataObjectCollection<Args...>::GetComponent;

    TopicDataObjectCollection(const std::string &topicName) :
        TopicDataObjectCollection<Args...>(topicName),
        m_TopicName(topicName)
    {
    }

    std::string Name() const {
        return m_TopicName;
    }

    static void SetComponent(std::shared_ptr<ITopicComponentDataObject> ptr, MaceCore::TopicDatagram &datagram) {
        if(std::dynamic_pointer_cast<T>(ptr) != 0) {
            SetComponent(std::dynamic_pointer_cast<T>(ptr), datagram);
        }
        else {
            TopicDataObjectCollection<Args...>::SetComponent(ptr, datagram);
        }
    }


    static bool GetComponent(std::shared_ptr<ITopicComponentDataObject> ptr, const MaceCore::TopicDatagram &datagram) {
        if(std::dynamic_pointer_cast<T>(ptr) != 0) {
            return GetComponent(datagram, std::dynamic_pointer_cast<T>(ptr));
        }
        else {
            return TopicDataObjectCollection<Args...>::GetComponent(ptr, datagram);
        }

    }

    static void SetComponent(const std::shared_ptr<T> &component, MaceCore::TopicDatagram &datagram) {
        datagram.AddNonTerminal(T::Name(), component->GenerateDatagram());
    }


    static bool GetComponent(const MaceCore::TopicDatagram &datagram, std::shared_ptr<T> value) {
        if(datagram.HasNonTerminal(T::Name()) == false) {
            return false;
        }

        value->CreateFromDatagram(*datagram.GetNonTerminal(T::Name()));
        return true;
    }

private:
    std::string m_TopicName;

};

}

#endif // TOPIC_COMPONENT_COLLECTION_H
