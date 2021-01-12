#ifndef ACTION_INITIALIZE_TEST_SETUP_H
#define ACTION_INITIALIZE_TEST_SETUP_H

#include <string>

#include "base/pose/geodetic_position_3D.h"
#include "base/pose/rotation_2D.h"
#include "base/pose/pose_basic_state.h"

#include "data/jsonconverter.h"
#include "abstract_ai_command.h"

namespace command_item {

class Action_InitializeTestSetup : public AbstractAICommand, public JSONConverter
{
public:
    Action_InitializeTestSetup();
    Action_InitializeTestSetup(const int &systemOrigin, const int &targetSystem);
    Action_InitializeTestSetup(const int &systemOrigin, const int &targetSystem, const mace::pose::BasicGeoState3D &state);
    Action_InitializeTestSetup(const Action_InitializeTestSetup &obj);

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

public:
    bool shouldTransitionToExecution() const
    {
        return _testRelease;
    }

public:
    void operator = (const Action_InitializeTestSetup &rhs)
    {
        AbstractAICommand::operator =(rhs);
        _testName = rhs._testName;
        _testRelease = rhs._testRelease;
        m_InitialConditions = rhs.m_InitialConditions;
    }

    bool operator == (const Action_InitializeTestSetup &rhs) {
        if(!AbstractAICommand::operator ==(rhs))
        {
            return false;
        }
        if(this->_testName != rhs._testName){
            return false;
        }
        if(this->m_InitialConditions != rhs.m_InitialConditions){
            return false;
        }
        if(_testRelease != rhs._testRelease)
        {
            return false;
        }
        return true;
    }

    bool operator != (const Action_InitializeTestSetup &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const Action_InitializeTestSetup &obj)
    {
        UNUSED(obj);
        return out;
    }

public:
    mace::pose::BasicGeoState3D m_InitialConditions;

private:
    std::string _testName; //some parameterized test reference

    bool _testRelease = true;
};

} //end of namespace command_item

#endif // ACTION_INITIALIZE_TEST_SETUP_H
