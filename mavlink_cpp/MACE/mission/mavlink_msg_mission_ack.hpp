// MESSAGE MISSION_ACK support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_ACK message
 *
 * This message is emitted after a mace has finished receiving a mission from another MACE instance. Seeing this response should cause MACE core to update the mission to the appropriate queue and notify other modules of the change based on the mission state received.
 */
struct MISSION_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 102;
    static constexpr size_t LENGTH = 7;
    static constexpr size_t MIN_LENGTH = 7;
    static constexpr uint8_t CRC_EXTRA = 87;
    static constexpr auto NAME = "MISSION_ACK";


    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t prev_mission_state; /*< The previous mission state, allowing us to recognize the original key. See MISSION_STATE */
    uint8_t mission_result; /*< The acknowledgement result associated, see MAV_MISSION_RESULT */
    uint8_t cur_mission_state; /*< The potential new mission state, see MISSION_STATE */


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
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  prev_mission_state: " << +prev_mission_state << std::endl;
        ss << "  mission_result: " << +mission_result << std::endl;
        ss << "  cur_mission_state: " << +cur_mission_state << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << mission_system;                // offset: 0
        map << mission_creator;               // offset: 1
        map << mission_id;                    // offset: 2
        map << mission_type;                  // offset: 3
        map << prev_mission_state;            // offset: 4
        map << mission_result;                // offset: 5
        map << cur_mission_state;             // offset: 6
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> mission_system;                // offset: 0
        map >> mission_creator;               // offset: 1
        map >> mission_id;                    // offset: 2
        map >> mission_type;                  // offset: 3
        map >> prev_mission_state;            // offset: 4
        map >> mission_result;                // offset: 5
        map >> cur_mission_state;             // offset: 6
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
