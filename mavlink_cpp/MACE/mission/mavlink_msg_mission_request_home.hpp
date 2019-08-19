// MESSAGE MISSION_REQUEST_HOME support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_REQUEST_HOME message
 *
 * The home position relating to the mission of the target system has been requested.
 */
struct MISSION_REQUEST_HOME : mavlink::Message {
    static constexpr msgid_t MSG_ID = 116;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 36;
    static constexpr auto NAME = "MISSION_REQUEST_HOME";


    uint8_t target_system; /*< The system which the home position is being requested from. */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
