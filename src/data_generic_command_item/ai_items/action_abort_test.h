#ifndef ACTION_ABORTTEST_H
#define ACTION_ABORTTEST_H

#include <string>
#include "base/pose/geodetic_position_3D.h"
#include "base/pose/rotation_2D.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"

namespace command_item {

class Action_AbortTest : public AbstractCommandItem
{
public:
    Action_AbortTest();
    Action_AbortTest(const Action_AbortTest &obj);
    Action_AbortTest(const int &systemOrigin, const int &targetSystem);

public:
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDTYPE getCommandType() const override;

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
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd) override;

    void generateMACEMSG_MissionItem(mace_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mace_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */


public:
    void operator = (const Action_AbortTest &rhs)
    {

    }

    bool operator == (const Action_AbortTest &rhs) {

        return true;
    }

    bool operator != (const Action_AbortTest &rhs) {
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
        //out<<"Command Arm( Request arm: "<<obj.actionArm<<")";
        return out;
    }

private:
    std::string descriptor;

};

} //end of namespace command_item

#endif // ACTION_ABORTTEST_H
