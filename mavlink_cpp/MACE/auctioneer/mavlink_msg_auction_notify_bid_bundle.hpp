// MESSAGE AUCTION_NOTIFY_BID_BUNDLE support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_NOTIFY_BID_BUNDLE message
 *
 * Notification that a bid bundle is going to be sent
 */
struct AUCTION_NOTIFY_BID_BUNDLE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10000;
    static constexpr size_t LENGTH = 17;
    static constexpr size_t MIN_LENGTH = 17;
    static constexpr uint8_t CRC_EXTRA = 72;
    static constexpr auto NAME = "AUCTION_NOTIFY_BID_BUNDLE";


    uint64_t agentID; /*< ID of agent generating the bundle */
    double bundleGenTime; /*< Bundle generation time */
    int8_t numBids; /*< Number of Bids */


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
        ss << "  numBids: " << +numBids << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << agentID;                       // offset: 0
        map << bundleGenTime;                 // offset: 8
        map << numBids;                       // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> agentID;                       // offset: 0
        map >> bundleGenTime;                 // offset: 8
        map >> numBids;                       // offset: 16
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
