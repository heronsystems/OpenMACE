// MESSAGE HEARTBEAT support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief HEARTBEAT message
 *
 * The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
 */
struct HEARTBEAT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 7;
    static constexpr size_t MIN_LENGTH = 7;
    static constexpr uint8_t CRC_EXTRA = 39;
    static constexpr auto NAME = "HEARTBEAT";


    uint8_t protocol; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) */
    uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM) */
    uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM */
    uint8_t mission_state; /*< Defines the current state of the vehicle mission. Useful for determing the next state of the vehicle per mission state. */
    uint8_t mace_companion; /*< Boolean describing whether(T=1) or not(F=0) the vehicle is MACE companion equipped. */
    uint8_t mavlink_version; /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version */
    uint8_t mavlinkID; /*<  */


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
        ss << "  protocol: " << +protocol << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  autopilot: " << +autopilot << std::endl;
        ss << "  mission_state: " << +mission_state << std::endl;
        ss << "  mace_companion: " << +mace_companion << std::endl;
        ss << "  mavlink_version: " << +mavlink_version << std::endl;
        ss << "  mavlinkID: " << +mavlinkID << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << protocol;                      // offset: 0
        map << type;                          // offset: 1
        map << autopilot;                     // offset: 2
        map << mission_state;                 // offset: 3
        map << mace_companion;                // offset: 4
        map << uint8_t(3);               // offset: 5
        map << mavlinkID;                     // offset: 6
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> protocol;                      // offset: 0
        map >> type;                          // offset: 1
        map >> autopilot;                     // offset: 2
        map >> mission_state;                 // offset: 3
        map >> mace_companion;                // offset: 4
        map >> mavlink_version;               // offset: 5
        map >> mavlinkID;                     // offset: 6
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
