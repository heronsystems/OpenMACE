#include "event_tag.h"

namespace Data {

EventTag::EventTag(const LOGGING_EVENT_TAGS &tag, const std::string &text)
{
    _tag = tag;
    _logText = text;
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, _time);
}

EventTag::EventTag(const EventTag &copy)
{
    _tag = copy._tag;
    _logText = copy._logText;
    _time = copy._time;
}

QJsonObject EventTag::toJSON(const int &vehicleID, const std::string &dataType) const{
    QJsonObject json = toJSON_base(vehicleID,dataType);
    json["event"] = LoggingEventToString(get_EventTag()).c_str();
    json["time"] = _time.ToQTDateTime().toString(Qt::ISODateWithMs);
    json["text"] = get_LoggingText().c_str();
    return json;
}

void EventTag::fromJSON(const QJsonDocument &inputJSON){

    _tag = LoggingEventFromString(inputJSON.object().value("event").toString().toStdString());
    _time.setTime(QDateTime::fromString(inputJSON.object().value("time").toString(),Qt::ISODateWithMs));
    _logText = inputJSON.object().value("text").toString().toStdString();
}

std::string EventTag::toCSV(const std::string &delimiter) const{
    std::string newline = LoggingEventToString(get_EventTag()) + delimiter + _time.ToQTDateTime().toString(Qt::ISODateWithMs).toStdString() + delimiter + get_LoggingText() + delimiter;
    return newline;
}

std::string EventTag::printCommandInfo() const
{
    return "";
}

} //end of namespace Data
