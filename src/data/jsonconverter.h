#ifndef JSONCONVERTER_H
#define JSONCONVERTER_H

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpSocket>

class JSONConverter
{
public:
    JSONConverter();
    ~JSONConverter();
    = default;
    
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const = 0;
    virtual QJsonObject toJSON_base(const int &vehicleID, const std::string &dataType) const
    {
        QJsonObject json;
        json["message_type"] = dataType;
        json["agentID"] = std::to_string(vehicleID).c_str();;
        return json;
    }
};

#endif