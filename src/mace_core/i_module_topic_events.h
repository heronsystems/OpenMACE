#ifndef I_MODULE_TOPIC_EVENTS_H
#define I_MODULE_TOPIC_EVENTS_H
#include <functional>
#include "vehicle_data.h"
#include "topic.h"

#include "abstract_module_base.h"

namespace MaceCore
{

class IModuleTopicEvents
{
public:

    //!
    //! \brief Subscribe to a topic being distrubted by MACE
    //! \param sender Pointer to object that is subscribing
    //! \param topicName Name of topic to subscribe to
    //! \param entityIDs Optional list of ids such that module will only be notified when a topic with entity of given ID is sent
    //! \param components Optional list of components such that module will only be notified when a component is updated
    //!
    virtual void Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &entityIDs = {}, const std::vector<std::string> &components = {}) = 0;

    //!
    //! \brief NewTopicDataValues New topic data values available
    //! \param moduleFrom Module the new data is available from
    //! \param topicName Topic name
    //! \param sender Sender module characteristic
    //! \param time Time stamp
    //! \param value Topic datagram value
    //! \param target Target module
    //!
    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const ModuleCharacteristic &sender, const TIME &time, const TopicDatagram &value, const OptionalParameter<ModuleCharacteristic> &target = OptionalParameter<ModuleCharacteristic>()) = 0;

    //!
    //! \brief Publish a new topic to MACE to distribute to all other modules
    //! \param topicName Name of topic
    //! \param entityID ID of entity that generated the topic
    //! \param time Time topic is valid for
    //! \param value Datagram for topic
    //!
    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int entityID, const TIME &time, const TopicDatagram &value) = 0;

};

} //End MaceCore Namespace


#endif // I_MODULE_TOPIC_EVENTS_H
