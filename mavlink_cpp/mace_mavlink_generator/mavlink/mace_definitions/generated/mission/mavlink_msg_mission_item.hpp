// MESSAGE MISSION_ITEM support class

#pragma once

namespace mavlink {
namespace mission {
namespace msg {

/**
 * @brief MISSION_ITEM message
 *
 * Message encoding a mission item. This message is emitted to announce
                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
 */
struct MISSION_ITEM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 107;
    static constexpr size_t LENGTH = 41;
    static constexpr size_t MIN_LENGTH = 41;
    static constexpr uint8_t CRC_EXTRA = 49;
    static constexpr auto NAME = "MISSION_ITEM";


    uint8_t target_system; /*< Target System ID */
    uint8_t mission_system; /*< Mission System ID */
    uint8_t mission_creator; /*< Creator ID */
    uint8_t mission_id; /*< Mission ID */
    uint8_t mission_type; /*< Mission type, see MISSION_TYPE */
    uint8_t mission_state; /*< The mission state, see MISSION_STATE */
    uint16_t seq; /*< Sequence */
    uint8_t frame; /*< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h */
    uint16_t command; /*< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs */
    uint8_t current; /*< false:0, true:1 */
    uint8_t autocontinue; /*< autocontinue to next wp */
    float param1; /*< PARAM1, see MAV_CMD enum */
    float param2; /*< PARAM2, see MAV_CMD enum */
    float param3; /*< PARAM3, see MAV_CMD enum */
    float param4; /*< PARAM4, see MAV_CMD enum */
    float x; /*< PARAM5 / local: x position, global: latitude */
    float y; /*< PARAM6 / y position: global: longitude */
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame. */


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
        ss << "  frame: " << +frame << std::endl;
        ss << "  command: " << command << std::endl;
        ss << "  current: " << +current << std::endl;
        ss << "  autocontinue: " << +autocontinue << std::endl;
        ss << "  param1: " << param1 << std::endl;
        ss << "  param2: " << param2 << std::endl;
        ss << "  param3: " << param3 << std::endl;
        ss << "  param4: " << param4 << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << param1;                        // offset: 0
        map << param2;                        // offset: 4
        map << param3;                        // offset: 8
        map << param4;                        // offset: 12
        map << x;                             // offset: 16
        map << y;                             // offset: 20
        map << z;                             // offset: 24
        map << seq;                           // offset: 28
        map << command;                       // offset: 30
        map << target_system;                 // offset: 32
        map << mission_system;                // offset: 33
        map << mission_creator;               // offset: 34
        map << mission_id;                    // offset: 35
        map << mission_type;                  // offset: 36
        map << mission_state;                 // offset: 37
        map << frame;                         // offset: 38
        map << current;                       // offset: 39
        map << autocontinue;                  // offset: 40
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> param1;                        // offset: 0
        map >> param2;                        // offset: 4
        map >> param3;                        // offset: 8
        map >> param4;                        // offset: 12
        map >> x;                             // offset: 16
        map >> y;                             // offset: 20
        map >> z;                             // offset: 24
        map >> seq;                           // offset: 28
        map >> command;                       // offset: 30
        map >> target_system;                 // offset: 32
        map >> mission_system;                // offset: 33
        map >> mission_creator;               // offset: 34
        map >> mission_id;                    // offset: 35
        map >> mission_type;                  // offset: 36
        map >> mission_state;                 // offset: 37
        map >> frame;                         // offset: 38
        map >> current;                       // offset: 39
        map >> autocontinue;                  // offset: 40
    }
};

} // namespace msg
} // namespace mission
} // namespace mavlink
