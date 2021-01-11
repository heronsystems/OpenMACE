#ifndef EVENT_TAG_H
#define EVENT_TAG_H

#include <string>
#include <mavlink.h>

#include "common/common.h"

#include "data/environment_time.h"
#include "data/jsonconverter.h"

namespace Data {

inline std::string LoggingEventToString(const LOGGING_EVENT_TAGS &eventType) {
    switch (eventType) {
    case LOGGING_EVENT_TAGS::NEW_TEST_EVALUATION:
        return "NEW_TEST_EVALUATION";
    case LOGGING_EVENT_TAGS::ROUTING_TO_INITIALIZATION:
        return "ROUTING_TO_INITIALIZATION";
    case LOGGING_EVENT_TAGS::ENABLED_CONTROL:
        return "ENABLED_CONTROL";
    case LOGGING_EVENT_TAGS::INTERESTING_OBSERVATION:
        return "INTERESTING_OBSERVATION";
    case LOGGING_EVENT_TAGS::ABORTED_TEST:
        return "ABORTED_TEST";
    case LOGGING_EVENT_TAGS::CEASING_TEST_EVALUATION:
        return "CEASING_TEST_EVALUATION";
    case LOGGING_EVENT_TAGS::GENERIC_ENTRY:
        return "GENERIC_ENTRY";
    default:
        throw std::runtime_error("Unknown LOGGING_EVENT_TAG seen");
    }
}

inline LOGGING_EVENT_TAGS LoggingEventFromString(const std::string &str) {
    if(str == "NEW_TEST_EVALUATION")
        return LOGGING_EVENT_TAGS::NEW_TEST_EVALUATION;
    if(str == "ROUTING_TO_INITIALIZATION")
        return LOGGING_EVENT_TAGS::ROUTING_TO_INITIALIZATION;
    if(str == "ENABLED_CONTROL")
        return LOGGING_EVENT_TAGS::ENABLED_CONTROL;
    if(str == "INTERESTING_OBSERVATION")
        return LOGGING_EVENT_TAGS::INTERESTING_OBSERVATION;
    if(str == "ABORTED_TEST")
        return LOGGING_EVENT_TAGS::ABORTED_TEST;
    if(str == "CEASING_TEST_EVALUATION")
        return LOGGING_EVENT_TAGS::CEASING_TEST_EVALUATION;
    if(str == "GENERIC_ENTRY")
        return LOGGING_EVENT_TAGS::GENERIC_ENTRY;
    throw std::runtime_error("Unknown string LOGGING_EVENT_TAGS seen");
}
class EventTag: public JSONConverter
{
public:
    enum class TAGGING_DEPTH : uint8_t{
        LOCAL = 0,
        REMOTE = 1,
        ALL = 2
    };

public:
    EventTag(const LOGGING_EVENT_TAGS &tag = LOGGING_EVENT_TAGS::GENERIC_ENTRY, const std::string &text = "");

    EventTag(const EventTag &copy);

    virtual ~EventTag() = default;

public:
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const override;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;

public:
    LOGGING_EVENT_TAGS get_EventTag() const
    {
        return _tag;
    }

    std::string get_LoggingText() const
    {
        return _logText;
    }

public:
    void operator = (const EventTag &rhs)
    {
        _time = rhs._time;
        _tag = rhs._tag;
        _logText = rhs._logText;
    }

    bool operator == (const EventTag &rhs) {
        if(_time != rhs._time)
        {
            return false;
        }
        if(_tag != rhs._tag)
        {
            return false;
        }
        if(_logText != rhs._logText)
        {
            return false;
        }
        return true;
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const;

    friend std::ostream &operator<<(std::ostream &out, const EventTag &obj)
    {
        UNUSED(obj);
        return out;
    }

private:
    Data::EnvironmentTime _time;
    LOGGING_EVENT_TAGS _tag;
    std::string _logText;
};

} //end of namespace data

#endif // EVENT_TAG_H
