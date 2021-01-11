#ifndef ACTION_ABORTTEST_H
#define ACTION_ABORTTEST_H

#include <string>

#include "mavlink.h"

#include "abstract_ai_command.h"

#include "data/jsonconverter.h"

namespace command_item {

inline std::string ProceduralCommandToString(const AI_PROCEDURAL_COMMANDS &eventType) {
    switch (eventType) {
    case AI_PROCEDURAL_COMMANDS::RELEASE:
        return "RELEASE";
    case AI_PROCEDURAL_COMMANDS::START:
        return "START";
    case AI_PROCEDURAL_COMMANDS::STOP:
        return "STOP";
    case AI_PROCEDURAL_COMMANDS::ABORT:
        return "ABORT";
    default:
        throw std::runtime_error("Unknown AI_PROCEDURAL_COMMAND seen");
    }
}

inline AI_PROCEDURAL_COMMANDS ProceduralCommandFromString(const std::string &str) {
    if(str == "RELEASE")
        return AI_PROCEDURAL_COMMANDS::RELEASE;
    if(str == "START")
        return AI_PROCEDURAL_COMMANDS::START;
    if(str == "STOP")
        return AI_PROCEDURAL_COMMANDS::STOP;
    if(str == "ABORT")
        return AI_PROCEDURAL_COMMANDS::ABORT;
    throw std::runtime_error("Unknown string AI_PROCEDURAL_COMMANDS seen");
}

class Action_ProceduralCommand : public AbstractAICommand, public JSONConverter
{
public:
    Action_ProceduralCommand();
    Action_ProceduralCommand(const Action_ProceduralCommand &obj);
    Action_ProceduralCommand(const int &systemOrigin, const int &targetSystem);

public:
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const override;

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
     * @brief setDescriptor
     * @return
     */
    void setDescriptor(const std::string description);

    /**
     * @brief setProcedural
     * @return
     */
    void setProcedural(const AI_PROCEDURAL_COMMANDS description);

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

    AI_PROCEDURAL_COMMANDS whatIsTheProcedural() const
    {
        return _procedural;
    }

public:
    void operator = (const Action_ProceduralCommand &rhs)
    {
        AbstractAICommand::operator =(rhs);
        _procedural = rhs._procedural;
        descriptor = rhs.descriptor;
    }

    bool operator == (const Action_ProceduralCommand &rhs) {
        if(!AbstractAICommand::operator ==(rhs))
        {
            return false;
        }
        if(_procedural != rhs._procedural)
        {
            return false;
        }
        return true;
    }

    bool operator != (const Action_ProceduralCommand &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const Action_ProceduralCommand &obj)
    {
        UNUSED(obj);
        //out<<"Command Arm( Request arm: "<<obj.actionArm<<")";
        return out;
    }

private:
    std::string descriptor;
    AI_PROCEDURAL_COMMANDS _procedural;
};

} //end of namespace command_item

#endif // ACTION_ABORTTEST_H
