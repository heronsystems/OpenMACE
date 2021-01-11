#ifndef DATA_GENERIC_ITEM_TEXT_H
#define DATA_GENERIC_ITEM_TEXT_H

#include <iostream>
#include <string>
#include <stdexcept>

#include <mavlink.h>

namespace DataGenericItem {

class DataGenericItem_Text
{
public:

    static std::string StatusSeverityToString(const MAV_SEVERITY &state) {
        switch (state) {
        case MAV_SEVERITY::MAV_SEVERITY_EMERGENCY:
            return "EMERGENCY";
        case MAV_SEVERITY::MAV_SEVERITY_ALERT:
            return "ALERT";
        case MAV_SEVERITY::MAV_SEVERITY_CRITICAL:
            return "CRITICAL";
        case MAV_SEVERITY::MAV_SEVERITY_ERROR:
            return "ERROR";
        case MAV_SEVERITY::MAV_SEVERITY_WARNING:
            return "WARNING";
        case MAV_SEVERITY::MAV_SEVERITY_NOTICE:
            return "NOTICE";
        case MAV_SEVERITY::MAV_SEVERITY_INFO:
            return "INFO";
        case MAV_SEVERITY::MAV_SEVERITY_DEBUG:
            return "DEBUG";
        default:
            throw std::runtime_error("Unknown status severity seen");
        }
    }

public:
    DataGenericItem_Text();

    DataGenericItem_Text(const DataGenericItem_Text &copyObj);

    DataGenericItem_Text(const mavlink_statustext_t &copyObj);

public:

    void setText(const std::string &dataString){
        this->_dataString = dataString;
    }
    std::string getText() const{
        return _dataString;
    }

    void setSeverity(const MAV_SEVERITY &severity){
        this->_severity = severity;
    }

    MAV_SEVERITY getSeverity() const{
        return _severity;
    }

    mavlink_statustext_t getMACECommsObject() const;
    mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const;

public:
    void operator = (const DataGenericItem_Text &rhs)
    {
        this->_severity = rhs._severity;
        this->_dataString = rhs._dataString;
    }

    bool operator == (const DataGenericItem_Text &rhs) {
        if(this->_severity != rhs._severity){
            return false;
        }
        if(this->_dataString != rhs._dataString){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_Text &rhs) {
        return !(*this == rhs);
    }

protected:
    MAV_SEVERITY _severity;
    std::string _dataString;
};

} //end of namespace DataGenericItem

#endif // DATA_GENERIC_ITEM_TEXT_H
