// MESSAGE EXECUTE_SPATIAL_ACTION_ACK support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief EXECUTE_SPATIAL_ACTION_ACK message
 *
 * Acknowledgement of a goto command.
 */
struct EXECUTE_SPATIAL_ACTION_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 36;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 108;
    static constexpr auto NAME = "EXECUTE_SPATIAL_ACTION_ACK";


    uint8_t result; /*< See MAV_RESULT enum */


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
        ss << "  result: " << +result << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << result;                        // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> result;                        // offset: 0
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
