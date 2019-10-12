// MESSAGE MISSION_REQUEST_LIST_GENERIC support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_REQUEST_LIST_GENERIC message
 *
 * Request the mission related from the target system with the appropirate mission type and state. The response to this message should be MISSION_COUNT.
 */
struct MISSION_REQUEST_LIST_GENERIC : mavlink::Message {
    static constexpr msgid_t MSG_ID = 103;
    static constexpr size_t LENGTH = 3;
    static constexpr size_t MIN_LENGTH = 3;
    static constexpr uint8_t CRC_EXTRA = 135;
    static constexpr auto NAME = "MISSION_REQUEST_LIST_GENERIC";


    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< The potential new mission state, see MISSION_STATE */


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
        ss << "  mission_system: " << +mission_system << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  mission_state: " << +mission_state << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << mission_system;                // offset: 0
        map << mission_type;                  // offset: 1
        map << mission_state;                 // offset: 2
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> mission_system;                // offset: 0
        map >> mission_type;                  // offset: 1
        map >> mission_state;                 // offset: 2
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
