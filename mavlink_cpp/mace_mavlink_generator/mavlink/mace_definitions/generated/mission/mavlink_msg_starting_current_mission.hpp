// MESSAGE STARTING_CURRENT_MISSION support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief STARTING_CURRENT_MISSION message
 *
 * This message is emitted when an aircraft starts a new mission. This could be caused by a mode change, or new mission as sent via a control module. The response to this may vary depending upon the knowledge and/or state of the receiving system. Systems recieving this system either acknowledge its existance in comparison with items in its queue, or, if further information is required, a mission request can be sent.
 */
struct STARTING_CURRENT_MISSION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 110;
    static constexpr size_t LENGTH = 4;
    static constexpr size_t MIN_LENGTH = 4;
    static constexpr uint8_t CRC_EXTRA = 224;
    static constexpr auto NAME = "STARTING_CURRENT_MISSION";


    uint8_t mission_system; /*< Mission System ID */
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
        ss << "  mission_system: " << +mission_system << std::endl;
        ss << "  mission_creator: " << +mission_creator << std::endl;
        ss << "  mission_id: " << +mission_id << std::endl;
        ss << "  mission_type: " << +mission_type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << mission_system;                // offset: 0
        map << mission_creator;               // offset: 1
        map << mission_id;                    // offset: 2
        map << mission_type;                  // offset: 3
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> mission_system;                // offset: 0
        map >> mission_creator;               // offset: 1
        map >> mission_id;                    // offset: 2
        map >> mission_type;                  // offset: 3
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
