// MESSAGE COMMAND_SHORT support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief COMMAND_SHORT message
 *
 * A short command message for those messages only requiring one parameter. Th
 */
struct COMMAND_SHORT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 31;
    static constexpr size_t LENGTH = 9;
    static constexpr size_t MIN_LENGTH = 9;
    static constexpr uint8_t CRC_EXTRA = 177;
    static constexpr auto NAME = "COMMAND_SHORT";


    uint16_t command; /*< Command ID, as defined by UXV_CMD enum.is was established to reduce the bandwidth required of messages not requiring as much parameterized data. */
    uint8_t target_system; /*< System which should execute the command */
    uint8_t target_component; /*< Component which should execute the command, 0 for all components */
    uint8_t confirmation; /*< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
    float param; /*< Parameter as defined by UXV_CMD enum. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  command: " << command << std::endl;
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  confirmation: " << +confirmation << std::endl;
        ss << "  param: " << param << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << param;                         // offset: 0
        map << command;                       // offset: 4
        map << target_system;                 // offset: 6
        map << target_component;              // offset: 7
        map << confirmation;                  // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> param;                         // offset: 0
        map >> command;                       // offset: 4
        map >> target_system;                 // offset: 6
        map >> target_component;              // offset: 7
        map >> confirmation;                  // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
