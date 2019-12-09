// MESSAGE AUCTION_TASK_STATUS support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_STATUS message
 *
 * Notification that a task was completed
 */
struct AUCTION_TASK_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10009;
    static constexpr size_t LENGTH = 82;
    static constexpr size_t MIN_LENGTH = 82;
    static constexpr uint8_t CRC_EXTRA = 91;
    static constexpr auto NAME = "AUCTION_TASK_STATUS";


    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    uint8_t type; /*< Status update type */
    uint64_t agentID; /*< Agent which is setting the status */
    std::array<uint8_t, 64> data; /*< Optional data dependent on type, up to 64 bytes. Set unused bytes to 0, to allow MAVLink v2 to truncate the message. Must be manually set to network byte order */


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
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  agentID: " << agentID << std::endl;
        ss << "  data: [" << to_string(data) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << creatorID;                     // offset: 0
        map << agentID;                       // offset: 8
        map << taskID;                        // offset: 16
        map << type;                          // offset: 17
        map << data;                          // offset: 18
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> creatorID;                     // offset: 0
        map >> agentID;                       // offset: 8
        map >> taskID;                        // offset: 16
        map >> type;                          // offset: 17
        map >> data;                          // offset: 18
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
