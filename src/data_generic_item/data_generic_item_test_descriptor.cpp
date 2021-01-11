#include "data_generic_item_test_descriptor.h"

namespace DataGenericItem{


void TestDescriptor::setRoundInfo(const int key, const QJsonDocument &data) {
    _testName = data.object().value("name").toString().toStdString();
    m_key.testID = key;
    _testDescription = data.object().value("description").toString().toStdString();
}

QJsonObject TestDescriptor::toJSON(const int &vehicleID, const std::string &dataType) const{
    QJsonObject json = toJSON_base(vehicleID,dataType);
    json["testID"] = m_key.testID;
    json["name"] = QString::fromStdString(getName());
    json["description"] = QString::fromStdString(getDescription());
    json["blueType"] = AdeptModelToString(m_key.blueAgent).c_str();
    json["redType"] = AdeptModelToString(m_key.redAgent).c_str();
    return json;
}

void TestDescriptor::fromJSON(const QJsonDocument &inputJSON){
    this->setRoundInfo(inputJSON.object().value("testID").toInt(),inputJSON);
}

std::string TestDescriptor::toCSV(const std::string &delimiter) const{
    std::string newline = std::to_string(m_key.testID) + delimiter + getName() + delimiter + getDescription() + delimiter;
    return newline;
}

void TestDescriptor::setKeyModel(const Data::DogfightTeam &team, const AdeptModelType &model){
    if(team == Data::DogfightTeam::RED){
        m_key.redAgent = model;
    } else if (team == Data::DogfightTeam::BLUE){
        m_key.blueAgent = model;
    }
}
} //end of namespace DataGenericItem
