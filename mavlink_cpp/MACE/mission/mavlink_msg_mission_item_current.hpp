// MESSAGE MISSION_ITEM_CURRENT support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_ITEM_CURRENT message
 *
 * Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
 */
struct MISSION_ITEM_CURRENT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 112;
    static constexpr size_t LENGTH = 7;
    static constexpr size_t MIN_LENGTH = 7;
    static constexpr uint8_t CRC_EXTRA = 47;
    static constexpr auto NAME = "MISSION_ITEM_CURRENT";


    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< The potential new mission state, see MISSION_STATE */
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
        map << mission_system;                // offset: 2
        map << mission_creator;               // offset: 3
        map << mission_id;                    // offset: 4
        map << mission_type;                  // offset: 5
        map << mission_state;                 // offset: 6
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> mission_system;                // offset: 2
        map >> mission_creator;               // offset: 3
        map >> mission_id;                    // offset: 4
        map >> mission_type;                  // offset: 5
        map >> mission_state;                 // offset: 6
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
