// MESSAGE COMMAND_SYSTEM_MODE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief COMMAND_SYSTEM_MODE message
 *
 * Report status of a command. Includes feedback wether the command was executed.
 */
struct COMMAND_SYSTEM_MODE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 33;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 129;
    static constexpr auto NAME = "COMMAND_SYSTEM_MODE";


    uint8_t target_system; /*< System which should execute the command */
    std::array<char, 20> mode; /*< Char string of the desired flight mode. 20 char limit. */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  mode: \"" << to_string(mode) << "\"" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
        map << mode;                          // offset: 1
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
        map >> mode;                          // offset: 1
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
