#include "action_initialize_test_setup.h"

namespace command_item {

COMMANDTYPE_AI Action_InitializeTestSetup::getCommandType() const
{
    return COMMANDTYPE_AI::CAI_ACT_TEST_INITIALIZATION;
}

std::string Action_InitializeTestSetup::getDescription() const
{
    return "This command will tell the vehicle to get ready for a test evaluation";
}

bool Action_InitializeTestSetup::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractAICommand> Action_InitializeTestSetup::getClone() const
{
    return std::make_shared<Action_InitializeTestSetup>(*this);
}

void Action_InitializeTestSetup::getClone(std::shared_ptr<AbstractAICommand> &command) const
{
    command = std::make_shared<Action_InitializeTestSetup>(*this);
}

QJsonObject Action_InitializeTestSetup::toJSON(const int &vehicleID, const std::string &dataType) const{
    UNUSED(dataType);
    QJsonObject json = toJSON_base(vehicleID,AICommandItemToString(getCommandType()));
    json["name"] = _testName.c_str();
    QJsonObject initialCondition;
    initialCondition["lat"] = m_InitialConditions.getPosition().getLatitude();
    initialCondition["lng"] = m_InitialConditions.getPosition().getLongitude();
    initialCondition["alt"] = m_InitialConditions.getPosition().getAltitude();
    initialCondition["roll"] = m_InitialConditions.getRotation().getRoll() * (180/M_PI);
    initialCondition["pitch"] = m_InitialConditions.getRotation().getPitch() * (180/M_PI);
    double yaw = m_InitialConditions.getRotation().getYaw();
    initialCondition["yaw"] = (180/M_PI)*(yaw < 0 ? yaw + 2*M_PI : yaw);
    initialCondition["speed"] = m_InitialConditions.getSpeed();
    json["inital_condition"] = initialCondition;
    return json;
}

void Action_InitializeTestSetup::fromJSON(const QJsonDocument &inputJSON){
    _testName = inputJSON.object().value("name").toString().toStdString();
    QJsonObject ICobject = inputJSON.object().value("initial_condition").toObject();
    mace::pose::GeodeticPosition_3D pose(ICobject.value("lat").toDouble(),ICobject.value("lng").toDouble(),ICobject.value("alt").toDouble());
    mace::pose::Rotation_3D att(ICobject.value("roll").toDouble()*(M_PI/180),ICobject.value("pitch").toDouble()*(M_PI/180),ICobject.value("yaw").toDouble()*(M_PI/180));
    m_InitialConditions.setPosition(pose);
    m_InitialConditions.setRotation(att);
    m_InitialConditions.setSpeed(ICobject.value("speed").toDouble());

}

std::string Action_InitializeTestSetup::toCSV(const std::string &delimiter) const{
    std::string newline = _testName + delimiter + std::to_string(m_InitialConditions.getPosition().getLatitude())+ delimiter + std::to_string(m_InitialConditions.getPosition().getLongitude())+ delimiter + std::to_string(m_InitialConditions.getPosition().getAltitude()) + delimiter;
    double yaw = m_InitialConditions.getRotation().getYaw();
    newline += std::to_string(m_InitialConditions.getRotation().getRoll()*(180/M_PI))+ delimiter + std::to_string(m_InitialConditions.getRotation().getPitch()*(180/M_PI))+ delimiter  + std::to_string((180/M_PI)*(yaw < 0 ? yaw + 2*M_PI : yaw))+ delimiter + std::to_string(m_InitialConditions.getSpeed()) + delimiter;
    return newline;
}

Action_InitializeTestSetup::Action_InitializeTestSetup()
{

}

Action_InitializeTestSetup::Action_InitializeTestSetup(const Action_InitializeTestSetup &obj):
    AbstractAICommand(obj)
{
    this->operator =(obj);
}

Action_InitializeTestSetup::Action_InitializeTestSetup(const int &systemOrigin, const int &targetSystem):
    AbstractAICommand(systemOrigin, targetSystem)
{

}

Action_InitializeTestSetup::Action_InitializeTestSetup(const int &systemOrigin, const int &targetSystem, const mace::pose::BasicGeoState3D &state):
    AbstractAICommand(systemOrigin, targetSystem)
{
    m_InitialConditions = state;
}


std::string Action_InitializeTestSetup::printCommandInfo() const
{
    return "";
}

} //end of namespace command_item

