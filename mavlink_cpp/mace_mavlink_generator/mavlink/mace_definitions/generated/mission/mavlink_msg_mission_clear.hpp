// MESSAGE MISSION_CLEAR support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_CLEAR message
 *
 * Delete all mission items at once.
 */
struct MISSION_CLEAR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 114;
    static constexpr size_t LENGTH = 4;
    static constexpr size_t MIN_LENGTH = 4;
    static constexpr uint8_t CRC_EXTRA = 34;
    static constexpr auto NAME = "MISSION_CLEAR";


    uint8_t target_system; /*< System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */


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
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << target_system;                 // offset: 0
        map << mission_creator;               // offset: 1
        map << mission_id;                    // offset: 2
        map << mission_type;                  // offset: 3
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> target_system;                 // offset: 0
        map >> mission_creator;               // offset: 1
        map >> mission_id;                    // offset: 2
        map >> mission_type;                  // offset: 3
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
