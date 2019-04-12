#ifndef DATA_GENERIC_ITEM_TEXT_H
#define DATA_GENERIC_ITEM_TEXT_H

#include <iostream>
#include <string>
#include <stdexcept>

#include "mace.h"

namespace DataGenericItem {

class DataGenericItem_Text
{
public:
    enum class STATUS_SEVERITY{
        STATUS_EMERGENCY,
        STATUS_ALERT,
        STATUS_CRITICAL,
        STATUS_ERROR,
        STATUS_WARNING,
        STATUS_NOTICE,
        STATUS_INFO,
        STATUS_DEBUG
    };

    static std::string StatusSeverityToString(const STATUS_SEVERITY &state) {
        switch (state) {
        case STATUS_SEVERITY::STATUS_EMERGENCY:
            return "EMERGENCY";
        case STATUS_SEVERITY::STATUS_ALERT:
            return "ALERT";
        case STATUS_SEVERITY::STATUS_CRITICAL:
            return "CRITICAL";
        case STATUS_SEVERITY::STATUS_ERROR:
            return "ERROR";
        case STATUS_SEVERITY::STATUS_WARNING:
            return "WARNING";
        case STATUS_SEVERITY::STATUS_NOTICE:
            return "NOTICE";
        case STATUS_SEVERITY::STATUS_INFO:
            return "INFO";
        case STATUS_SEVERITY::STATUS_DEBUG:
            return "DEBUG";
        default:
            throw std::runtime_error("Unknown status severity seen");
        }
    }
public:
    DataGenericItem_Text();

    DataGenericItem_Text(const DataGenericItem_Text &copyObj);

    DataGenericItem_Text(const mace_statustext_t &copyObj);

public:

    void setText(const std::string &dataString){
        this->dataString = dataString;
    }
    std::string getText() const{
        return dataString;
    }

    void setSeverity(const STATUS_SEVERITY &severity){
        this->severity = severity;
    }

    STATUS_SEVERITY getSeverity() const{
        return severity;
    }

    mace_statustext_t getMACECommsObject() const;
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_Text &rhs)
    {
        this->severity = rhs.severity;
        this->dataString = rhs.dataString;
    }

    bool operator == (const DataGenericItem_Text &rhs) {
        if(this->severity != rhs.severity){
            return false;
        }
        if(this->dataString != rhs.dataString){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Text &rhs) {
        return !(*this == rhs);
    }

protected:
    STATUS_SEVERITY severity;
    std::string dataString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_TEXT_H
