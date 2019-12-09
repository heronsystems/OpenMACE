// MESSAGE BOUNDARY_REQUEST_ITEM support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief BOUNDARY_REQUEST_ITEM message
 *
 * 
 */
struct BOUNDARY_REQUEST_ITEM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 134;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 138;
    static constexpr auto NAME = "BOUNDARY_REQUEST_ITEM";


    uint8_t boundary_host_sysid; /*< System ID */
    uint8_t boundary_host_compid; /*< Creator ID */
    uint8_t boundary_identifier; /*< Number to identifiy boundary on host. */
    uint16_t seq; /*< Item index within the bonudary sequence. */


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
        ss << "  seq: " << seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
        map << boundary_host_sysid;           // offset: 2
        map << boundary_host_compid;          // offset: 3
        map << boundary_identifier;           // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
        map >> boundary_host_sysid;           // offset: 2
        map >> boundary_host_compid;          // offset: 3
        map >> boundary_identifier;           // offset: 4
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
