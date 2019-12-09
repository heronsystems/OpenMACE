// MESSAGE AUCTION_ACK_BID_BUNDLE support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_ACK_BID_BUNDLE message
 *
 * Acknowledgment that a bid bundle has been fully received
 */
struct AUCTION_ACK_BID_BUNDLE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10004;
    static constexpr size_t LENGTH = 16;
    static constexpr size_t MIN_LENGTH = 16;
    static constexpr uint8_t CRC_EXTRA = 150;
    static constexpr auto NAME = "AUCTION_ACK_BID_BUNDLE";


    uint64_t agentID; /*< ID of agent generating the bundle */
    double bundleGenTime; /*< Bundle generation time */


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
        ss << "  bundleGenTime: " << bundleGenTime << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << agentID;                       // offset: 0
        map << bundleGenTime;                 // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> agentID;                       // offset: 0
        map >> bundleGenTime;                 // offset: 8
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
