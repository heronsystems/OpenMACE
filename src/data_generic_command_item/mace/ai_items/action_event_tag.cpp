#include "action_event_tag.h"

namespace command_item {

COMMANDTYPE_AI Action_EventTag::getCommandType() const
{
    return COMMANDTYPE_AI::CAI_EVENT_TAG;
}

std::string Action_EventTag::getDescription() const
{
    return "This command will tell all listeners to tag something within their logs";
}

bool Action_EventTag::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractAICommand> Action_EventTag::getClone() const
{
    return std::make_shared<Action_EventTag>(*this);
}

void Action_EventTag::getClone(std::shared_ptr<AbstractAICommand> &command) const
{
    command = std::make_shared<Action_EventTag>(*this);
}


Action_EventTag::Action_EventTag(const LOGGING_EVENT_TAGS &tag, const std::string &text)
{
    _tag = tag;
    _logText = text;
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, _time);
}

Action_EventTag::Action_EventTag(const Action_EventTag &obj):
    AbstractAICommand(0,0)
{
    this->operator =(obj);
}

Action_EventTag::Action_EventTag(const int &systemOrigin, const int &systemTarget):
    AbstractAICommand(systemOrigin,systemTarget)
{

}

QJsonObject Action_EventTag::toJSON(const int &vehicleID, const std::string &dataType) const{
    UNUSED(dataType);
    QJsonObject json = toJSON_base(vehicleID,AICommandItemToString(getCommandType()));
    json["event"] = Data::LoggingEventToString(get_EventTag()).c_str();
    json["time"] = _time.ToQTDateTime().toString(Qt::ISODateWithMs);
    json["text"] = get_LoggingText().c_str();
    return json;
}

void Action_EventTag::fromJSON(const QJsonDocument &inputJSON){

    _tag = Data::LoggingEventFromString(inputJSON.object().value("event").toString().toStdString());
    _time.setTime(QDateTime::fromString(inputJSON.object().value("time").toString(),Qt::ISODateWithMs));
    _logText = inputJSON.object().value("text").toString().toStdString();
}

std::string Action_EventTag::toCSV(const std::string &delimiter) const{
    std::string newline = Data::LoggingEventToString(get_EventTag()) + delimiter + _time.ToQTDateTime().toString(Qt::ISODateWithMs).toStdString() + delimiter + get_LoggingText() + delimiter;
    return newline;
}

std::string Action_EventTag::printCommandInfo() const
{
    return "";
}

} //end of namespace command_item
