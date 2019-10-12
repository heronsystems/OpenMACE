// MESSAGE MISSION_REQUEST_ITEM support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_REQUEST_ITEM message
 *
 * Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
 */
struct MISSION_REQUEST_ITEM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 106;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 3;
    static constexpr auto NAME = "MISSION_REQUEST_ITEM";


    uint8_t target_system; /*< System ID */
    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< The mission state, see MISSION_STATE */
    uint16_t seq; /*< Sequence */


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
        ss << "  mission_system: " << +mission_system << std::endl;
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;
        ss << "  mission_state: " << +mission_state << std::endl;
        ss << "  seq: " << seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
        map << target_system;                 // offset: 2
        map << mission_system;                // offset: 3
        map << mission_creator;               // offset: 4
        map << mission_id;                    // offset: 5
        map << mission_type;                  // offset: 6
        map << mission_state;                 // offset: 7
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> target_system;                 // offset: 2
        map >> mission_system;                // offset: 3
        map >> mission_creator;               // offset: 4
        map >> mission_id;                    // offset: 5
        map >> mission_type;                  // offset: 6
        map >> mission_state;                 // offset: 7
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
