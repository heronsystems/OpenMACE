#include "action_procedural_command.h"

namespace command_item {

COMMANDTYPE_AI Action_ProceduralCommand::getCommandType() const
{
    return COMMANDTYPE_AI::CAI_PROCEDURAL_ACTION;
}

std::string Action_ProceduralCommand::getDescription() const
{
    return "This command will tell the vehicle to explicitly execute a specific phase of the AI evaluation";
}

bool Action_ProceduralCommand::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractAICommand> Action_ProceduralCommand::getClone() const
{
    return std::make_shared<Action_ProceduralCommand>(*this);
}

void Action_ProceduralCommand::getClone(std::shared_ptr<AbstractAICommand> &command) const
{
    command = std::make_shared<Action_ProceduralCommand>(*this);
}

QJsonObject Action_ProceduralCommand::toJSON(const int &vehicleID, const std::string &dataType) const{
    UNUSED(dataType);
    QJsonObject json = toJSON_base(vehicleID,AICommandItemToString(getCommandType()));
    json["type"] = ProceduralCommandToString(_procedural).c_str();
    json["descriptor"] = descriptor.c_str(); //subject to change?
    return json;
}

void Action_ProceduralCommand::fromJSON(const QJsonDocument &inputJSON){
    _procedural = ProceduralCommandFromString(inputJSON.object().value("type").toString().toStdString());
    descriptor = inputJSON.object().value("descriptor").toString().toStdString();
}

std::string Action_ProceduralCommand::toCSV(const std::string &delimiter) const{
    std::string newline = ProceduralCommandToString(_procedural) + delimiter + descriptor + delimiter;
    return newline;
}

/**
 * @brief setDescriptor
 * @return
 */
void Action_ProceduralCommand::setDescriptor(const std::string description)
{
    this->descriptor = description;
}

/**
 * @brief setProceduraldescriptiondescription
 * @return
 */
void Action_ProceduralCommand::setProcedural(const AI_PROCEDURAL_COMMANDS commandtype)
{
    this->_procedural = commandtype;
}

Action_ProceduralCommand::Action_ProceduralCommand()
{

}

Action_ProceduralCommand::Action_ProceduralCommand(const Action_ProceduralCommand &obj):
    AbstractAICommand(0,0)
{
    this->operator =(obj);
}

Action_ProceduralCommand::Action_ProceduralCommand(const int &systemOrigin, const int &systemTarget):
    AbstractAICommand(systemOrigin,systemTarget)
{

}

std::string Action_ProceduralCommand::printCommandInfo() const
{
    return "";
}

} //end of namespace command_item


