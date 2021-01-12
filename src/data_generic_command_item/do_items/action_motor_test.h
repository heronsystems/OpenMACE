#ifndef ACTION_MOTOR_TEST_H
#define ACTION_MOTOR_TEST_H

#include <iostream>

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"


namespace command_item {

class ActionMotorTest : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_long_t>
{

public:
    /**
     * @brief getCommandType
     * @return
     */
    MAV_CMD getCommandType() const override;

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

    /** Interface imposed via Interface_CommandItem<mavlink_command_int_t> */
public:
    void populateCommandItem(mavlink_command_long_t &obj) const override;

    void fromCommandItem(const mavlink_command_long_t &obj) override;

    /** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */


    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */

public:
    ActionMotorTest();
    ActionMotorTest(const ActionMotorTest &obj);
    ActionMotorTest(const int &systemOrigin, const int &systemTarget);

public:
    void setThrottleSource(const  MOTOR_TEST_THROTTLE_TYPE &throttleSource)
    {
        this->throttleSource = throttleSource;
    }

     MOTOR_TEST_THROTTLE_TYPE getThrottleSource() const
    {
        return throttleSource;
    }

    void setMotorPower(const uint16_t &power)
    {
        this->power = power;
    }

    uint16_t getMotorPower() const
    {
        return this->power;
    }

    void setMotorNumber(const uint16_t &motorNumber)
    {
        this->motorNumber = motorNumber;
    }

    uint16_t getMotorNumber() const
    {
        return motorNumber;
    }


    void setMotorDuration(const uint16_t &duration)
    {
        this->duration = duration;
    }

    uint16_t getMotorDuration() const
    {
        return this->duration;
    }
public:
    void operator = (const ActionMotorTest &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->throttleSource = rhs.throttleSource;
        this->power = rhs.power;
        this->motorNumber = rhs.motorNumber;
        this->duration = rhs.duration;
    }

    bool operator == (const ActionMotorTest &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->throttleSource != rhs.throttleSource){
            return false;
        }
        if(this->power != rhs.power){
            return false;
        }
        if(this->motorNumber != rhs.motorNumber){
            return false;
        }
        if(this->duration != rhs.duration){
            return false;
        }
        return true;
    }

    bool operator != (const ActionMotorTest &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const ActionMotorTest &obj)
    {
        out<<"Command Motor Test( Power: "<<obj.power<<", Motor Number:"<<obj.motorNumber<<", Duration:"<<obj.duration<<")";
        return out;
    }

private:
    MOTOR_TEST_THROTTLE_TYPE throttleSource;
    uint16_t power;
    uint16_t motorNumber;
    uint16_t duration;
};

} //end of namespace MissionItem
#endif // ACTION_MOTOR_TEST_H
