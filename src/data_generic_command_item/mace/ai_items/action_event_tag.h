#ifndef ACTION_EVENTTAG_H
#define ACTION_EVENTTAG_H

#include <string>
#include "mavlink.h"

#include "data/environment_time.h"
#include "data/event_tag.h"
#include "data/jsonconverter.h"

#include "abstract_ai_command.h"

namespace command_item {

class Action_EventTag : public AbstractAICommand, public JSONConverter
{
public:
    enum class TAGGING_DEPTH : uint8_t{
        LOCAL = 0,
        REMOTE = 1,
        ALL = 2
    };

public:
    Action_EventTag(const LOGGING_EVENT_TAGS &tag = LOGGING_EVENT_TAGS::GENERIC_ENTRY, const std::string &text = "");
    Action_EventTag(const Action_EventTag &obj);
    Action_EventTag(const int &systemOrigin, const int &targetSystem);

public:
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType = "") const override;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;

public:
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDTYPE_AI getCommandType() const override;

    /**
     * @brief getDescription
     * @return
     */
    std::string getDescription() const override;

    /**
     * @brief hasSpatialInfluence
     * @return
     */
    bool hasSpatialInfluence() const override;

    /**
     * @brief getClone
     * @return
     */
    std::shared_ptr<AbstractAICommand> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractAICommand> &command) const override;

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
    void operator = (const Action_EventTag &rhs)
    {
        AbstractAICommand::operator =(rhs);
        _time = rhs._time;
        _tag = rhs._tag;
        _logText = rhs._logText;
    }

    bool operator == (const Action_EventTag &rhs) {
        if(!AbstractAICommand::operator ==(rhs))
        {
            return false;
        }
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

    bool operator != (const Action_EventTag &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const Action_EventTag &obj)
    {
        UNUSED(obj);
        return out;
    }

private:
    Data::EnvironmentTime _time;
    LOGGING_EVENT_TAGS _tag;
    std::string _logText;
};

} //end of namespace command_item

#endif // ACTION_EVENTTAG_H
