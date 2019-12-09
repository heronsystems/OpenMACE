// MESSAGE BOUNDARY_COUNT support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief BOUNDARY_COUNT message
 *
 * 
 */
struct BOUNDARY_COUNT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 133;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 122;
    static constexpr auto NAME = "BOUNDARY_COUNT";


    uint8_t boundary_host_sysid; /*< System ID */
    uint8_t boundary_host_compid; /*< Creator ID */
    uint8_t boundary_identifier; /*< Number to identifiy boundary on host. */
    uint16_t count; /*< Number of items defining the boundary. */


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
        ss << "  count: " << count << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << count;                         // offset: 0
        map << boundary_host_sysid;           // offset: 2
        map << boundary_host_compid;          // offset: 3
        map << boundary_identifier;           // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> count;                         // offset: 0
        map >> boundary_host_sysid;           // offset: 2
        map >> boundary_host_compid;          // offset: 3
        map >> boundary_identifier;           // offset: 4
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
