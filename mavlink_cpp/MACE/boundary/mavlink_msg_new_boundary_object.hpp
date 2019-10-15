// MESSAGE NEW_BOUNDARY_OBJECT support class

#pragma once

namespace mavlink {
namespace boundary {
namespace msg {

/**
 * @brief NEW_BOUNDARY_OBJECT message
 *
 * 
 */
struct NEW_BOUNDARY_OBJECT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 130;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 52;
    static constexpr auto NAME = "NEW_BOUNDARY_OBJECT";


    uint8_t boundary_host_sysid; /*< System ID */
    uint8_t boundary_host_compid; /*< Creator ID */
    uint8_t boundary_type; /*< Boundary type, see BOUNDARY_TYPE. */
    uint8_t boundary_identifier; /*< Number to identifiy boundary on host. */
    uint8_t vehicle_aplicable; /*< The vehicle that boundary applies to. */
    uint8_t num_vehicles; /*< Number of vehicles that the boundary contains. */


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
        ss << "  boundary_type: " << +boundary_type << std::endl;
        ss << "  boundary_identifier: " << +boundary_identifier << std::endl;
        ss << "  vehicle_aplicable: " << +vehicle_aplicable << std::endl;
        ss << "  num_vehicles: " << +num_vehicles << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << boundary_host_sysid;           // offset: 0
        map << boundary_host_compid;          // offset: 1
        map << boundary_type;                 // offset: 2
        map << boundary_identifier;           // offset: 3
        map << vehicle_aplicable;             // offset: 4
        map << num_vehicles;                  // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> boundary_host_sysid;           // offset: 0
        map >> boundary_host_compid;          // offset: 1
        map >> boundary_type;                 // offset: 2
        map >> boundary_identifier;           // offset: 3
        map >> vehicle_aplicable;             // offset: 4
        map >> num_vehicles;                  // offset: 5
    }
};

} // namespace msg
} // namespace boundary
} // namespace mavlink
