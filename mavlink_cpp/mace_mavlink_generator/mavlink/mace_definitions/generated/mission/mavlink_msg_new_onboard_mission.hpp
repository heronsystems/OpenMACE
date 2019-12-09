// MESSAGE NEW_ONBOARD_MISSION support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief NEW_ONBOARD_MISSION message
 *
 * This message is emitted currently under one circumstances. A vehicle may emit this when its local planners have developed a new mission for the vehicle and have updated one of its queues. The count of the mission items in this queue is sent as its more direct method of enabling other mace instances for transmission
 */
struct NEW_ONBOARD_MISSION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 100;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 132;
    static constexpr auto NAME = "NEW_ONBOARD_MISSION";


    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< Mission type, see MISSION_STATE */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << mission_system;                // offset: 0
        map << mission_creator;               // offset: 1
        map << mission_id;                    // offset: 2
        map << mission_type;                  // offset: 3
        map << mission_state;                 // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> mission_system;                // offset: 0
        map >> mission_creator;               // offset: 1
        map >> mission_id;                    // offset: 2
        map >> mission_type;                  // offset: 3
        map >> mission_state;                 // offset: 4
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
