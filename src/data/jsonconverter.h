#ifndef JSONCONVERTER_H
#define JSONCONVERTER_H

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

class JSONConverter
{
public:
    JSONConverter()
    {
        
    }
    ~JSONConverter() = default;
    
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const = 0;
    virtual QJsonObject toJSON_base(const int &vehicleID, const std::string &dataType) const
    {
        QJsonObject json;
        json["message_type"] = dataType.c_str();
        json["agentID"] = std::to_string(vehicleID).c_str();;
        return json;
    }

//    virtual void fromJSON(const std::string &inputJSON) const = 0;
//    virtual std::string toCSV() const = 0;
};

#endif
