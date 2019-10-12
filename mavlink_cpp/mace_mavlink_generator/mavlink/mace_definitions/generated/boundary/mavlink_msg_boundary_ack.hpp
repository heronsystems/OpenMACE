// MESSAGE BOUNDARY_ACK support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief BOUNDARY_ACK message
 *
 * 
 */
struct BOUNDARY_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 131;
    static constexpr size_t LENGTH = 4;
    static constexpr size_t MIN_LENGTH = 4;
    static constexpr uint8_t CRC_EXTRA = 220;
    static constexpr auto NAME = "BOUNDARY_ACK";


    uint8_t boundary_host_sysid; /*< System ID */
    uint8_t boundary_host_compid; /*< Creator ID */
    uint8_t boundary_identifier; /*< Number to identifiy boundary on host. */
    uint8_t boundary_result; /*< The acknowledgement result associated, see BOUNDARY_RESULT. */


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
        ss << "  boundary_host_sysid: " << +boundary_host_sysid << std::endl;
        ss << "  boundary_host_compid: " << +boundary_host_compid << std::endl;
        ss << "  boundary_identifier: " << +boundary_identifier << std::endl;
        ss << "  boundary_result: " << +boundary_result << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << boundary_host_sysid;           // offset: 0
        map << boundary_host_compid;          // offset: 1
        map << boundary_identifier;           // offset: 2
        map << boundary_result;               // offset: 3
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> boundary_host_sysid;           // offset: 0
        map >> boundary_host_compid;          // offset: 1
        map >> boundary_identifier;           // offset: 2
        map >> boundary_result;               // offset: 3
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
