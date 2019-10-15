// MESSAGE AUCTION_BID_DESCRIPTOR_ACK support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_BID_DESCRIPTOR_ACK message
 *
 * Acknowledgment that a bid descriptor was received
 */
struct AUCTION_BID_DESCRIPTOR_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10012;
    static constexpr size_t LENGTH = 25;
    static constexpr size_t MIN_LENGTH = 25;
    static constexpr uint8_t CRC_EXTRA = 97;
    static constexpr auto NAME = "AUCTION_BID_DESCRIPTOR_ACK";


    uint64_t agentID; /*< Agent ID */
    double bidGenTime; /*< Bid generation time */
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
        ss << "  agentID: " << agentID << std::endl;
        ss << "  bidGenTime: " << bidGenTime << std::endl;
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << agentID;                       // offset: 0
        map << bidGenTime;                    // offset: 8
        map << creatorID;                     // offset: 16
        map << taskID;                        // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> agentID;                       // offset: 0
        map >> bidGenTime;                    // offset: 8
        map >> creatorID;                     // offset: 16
        map >> taskID;                        // offset: 24
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
