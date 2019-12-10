// MESSAGE AUCTION_REQUEST_TASK_DESCRIPTOR_ACK support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_REQUEST_TASK_DESCRIPTOR_ACK message
 *
 * Acknowledgment of the request for a task descriptor
 */
struct AUCTION_REQUEST_TASK_DESCRIPTOR_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10008;
    static constexpr size_t LENGTH = 9;
    static constexpr size_t MIN_LENGTH = 9;
    static constexpr uint8_t CRC_EXTRA = 200;
    static constexpr auto NAME = "AUCTION_REQUEST_TASK_DESCRIPTOR_ACK";


    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */


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

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << creatorID;                     // offset: 0
        map << taskID;                        // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> creatorID;                     // offset: 0
        map >> taskID;                        // offset: 8
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
