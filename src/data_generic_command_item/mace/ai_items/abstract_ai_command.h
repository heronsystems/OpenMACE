#ifndef ABSTRACT_AI_COMMAND_H
#define ABSTRACT_AI_COMMAND_H

#include <sstream>
#include <string>

#include "common/common.h"
#include "common/class_forward.h"

#include "ai_command_type.h"

namespace command_item {

MACE_CLASS_FORWARD(AbstractAICommand);

//!
//! \brief The AbstractAICommand class is an abstract class designed to hold the basic information required
//! for an abstract AI command item. It establishes real valued functions for assigning the originating and target
//! systems. Additionally, it provides virtual methods for getting parameters needed for rendering and
//! casting the command type.
//!
class AbstractAICommand
{
public:
    //!
    //! \brief AbstractCommandItem The default constructor used when this class is inherited. The default
    //! constructor shall assign the initial originating system and target system to IDs of 0. A developer
    //! should be aware that a target system of 0 could indicate that this command is for all connected
    //! vehicles within the MACE communications network. In the future it should be noted that a value
    //! of 0 for the originatingSystem may not be allowed and checked as this makes no conclusive sense
    //! in the MACE network.
    //!
    AbstractAICommand():
        originatingSystem(0), targetSystem(0), targetComponent(0)
    {

    }

    AbstractAICommand(const AbstractAICommand &copy)
    {
        this->originatingSystem = copy.originatingSystem;
        this->targetSystem = copy.targetSystem;
        this->targetComponent = copy.targetComponent;
    }

    //!
    //! \brief AbstractCommandItem Overloaded default constructor that requires the systemOrigin value argument
    //! to be passed while leaving the systemTarget parameter optional.
    //! \param systemOrigin The ID value of the system that initiated the command item.
    //! \param systemTarget The ID value of the system that is the intended recipient of the command item.
    //! A developer should be aware that this value defaults to 0 if no arguments are provided.
    //!
    AbstractAICommand(const unsigned int &systemOrigin, const unsigned int &systemTarget = 0, const unsigned int &componentTarget = 0):
        originatingSystem(systemOrigin), targetSystem(systemTarget), targetComponent(componentTarget)
    {

    }

    virtual ~AbstractAICommand()
    {

    }

public:

    //!
    //! \brief getCommandType returns the type of the object that this command type is.
    //! \return Data::CommandType resolving the type of command this object is.
    //!
    virtual COMMANDTYPE_AI getCommandType() const = 0;

    //!
    //! \brief hasSpatialInfluence returns a boolean reflecting whether or not the commandItem has
    //! a direct influence over a vehicles position. This is useful for determining flight times,
    //! position elements, or rendering objects on a GUI.
    //! \return false if the command does not have an affect over the vehicles position directly.
    //! For example, change speed has no influence over a vehicles position.
    //!
    virtual bool hasSpatialInfluence() const = 0;

    //!
    //! \brief getDescription
    //! \return string describing the command item. This may be useful for setting up options in a
    //! GUI or somewhere a display needs to interface and decisions have to be made describing what
    //! would happen when issuing such a command.
    //!
    virtual std::string getDescription() const = 0;

public:

    //!
    //! \brief setTargetSystem sets the targetSystem of the commandItem object.
    //! \param systemID value of the target system that should be receiving and enacting this command.
    //! A value of 0 here means that all systems will receive the command.
    //!
    void setTargetSystem(const unsigned int &systemID){
        targetSystem = systemID;
    }

    //!
    //! \brief getTargetSystem retrieves the int value of the targetSystem of the commandItem object;
    //! \return int value of the targetSystem member object.
    //!
    unsigned int getTargetSystem() const{
        return targetSystem;
    }

    //!
    //! \brief setOriginatingSystem sets the originatingSystem of the commandItem object.
    //! \param systemID int value of the originatingSystem member object.
    //!
    void setOriginatingSystem(const unsigned int &systemID){
        originatingSystem = systemID;
    }

    //!
    //! \brief getOriginatingSystem retries the int value of the originatingSystem of the commandItem object.
    //! \return int value of the originatingSystem member object.
    //!
    unsigned int getOriginatingSystem() const{
        return originatingSystem;
    }

public:
    /**
     *
     */
    template <class T>
    const T *as() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *as()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual std::shared_ptr<AbstractAICommand> getClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getClone(std::shared_ptr<AbstractAICommand> &command) const = 0;

public:

    //!
    //! \brief operator = overloaded assignment operator for AbstractCommandItems.
    //! \param rhs object that the data is copied from in the assignmnet operator.
    //!
    AbstractAICommand& operator = (const AbstractAICommand &rhs)
    {
        this->originatingSystem = rhs.originatingSystem;
        this->targetSystem = rhs.targetSystem;
        this->targetComponent = rhs.targetComponent;
        return *this;
    }

    //!
    //! \brief operator == overloaded comparison operator for AbstractCommandItems.
    //! \param rhs object that the data is compared against.
    //! \return true if the obejcts are equal.
    //!
    bool operator == (const AbstractAICommand &rhs) {
        if(this->originatingSystem != rhs.originatingSystem){
            return false;
        }
        if(this->targetSystem != rhs.targetSystem){
            return false;
        }
        if(this->targetComponent != rhs.targetComponent){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator != overloaded comparison operator for AbstractCommandItems.
    //! \param rhs object that the data is compared against.
    //! \return false if the objects are equal.
    //!
    bool operator != (const AbstractAICommand &rhs) {
        return !(*this == rhs);
    }

private:


public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    virtual std::string printCommandInfo() const = 0;

    //!
    //! \brief printPositionLog
    //! \param os
    //!
    virtual void printCommandLog(std::stringstream &stream) const
    {
        //stream << "AI_CMD|" <<std::to_string(originatingSystem)<<"|"<<std::to_string(targetSystem)<<"|"<<CommandItemToString(this->getCommandType())<<"[";
        stream << printCommandInfo();
        stream << "]";
    }

    friend std::ostream& operator<<(std::ostream& os, const AbstractAICommand* t)
    {
        std::stringstream newStream;
        t->printCommandLog(newStream);
        os << newStream.str();
        return os;
    }

protected:
    //!
    //! \brief originatingSystem value relating to the systemID of the system that had initially generated
    //! or transmitted the command item.
    //!
    unsigned int originatingSystem;

    //!
    //! \brief targetSystem value relating to the systemID of the system that should be receiving and enacting
    //! the command item.
    //!
    unsigned int targetSystem;

    //!
    //! \brief targetSystem value relating to the systemID of the system that should be receiving and enacting
    //! the command item.
    //!
    unsigned int targetComponent;
};

}
#endif // ABSTRACT_AI_COMMAND_H
