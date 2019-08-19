// MESSAGE NEW_PROPOSED_MISSION support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief NEW_PROPOSED_MISSION message
 *
 * This message is emitted currently under one circumstances. A GCS equipped station may emit this message in order to initiate a write transaction.
 */
struct NEW_PROPOSED_MISSION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 101;
    static constexpr size_t LENGTH = 7;
    static constexpr size_t MIN_LENGTH = 7;
    static constexpr uint8_t CRC_EXTRA = 18;
    static constexpr auto NAME = "NEW_PROPOSED_MISSION";


    uint8_t target_system; /*< System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< Mission type, see MISSION_STATE */
    uint16_t count; /*< Number of mission items in the sequence */


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
        ss << "  mission_state: " << +mission_state << std::endl;
        ss << "  count: " << count << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << count;                         // offset: 0
        map << target_system;                 // offset: 2
        map << mission_creator;               // offset: 3
        map << mission_id;                    // offset: 4
        map << mission_type;                  // offset: 5
        map << mission_state;                 // offset: 6
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> count;                         // offset: 0
        map >> target_system;                 // offset: 2
        map >> mission_creator;               // offset: 3
        map >> mission_id;                    // offset: 4
        map >> mission_type;                  // offset: 5
        map >> mission_state;                 // offset: 6
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
