// MESSAGE AUCTION_BID_BUNDLE_RCV_PART support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_BID_BUNDLE_RCV_PART message
 *
 * Confirmation that the last bid bundle part was received. 
 */
struct AUCTION_BID_BUNDLE_RCV_PART : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10002;
    static constexpr size_t LENGTH = 17;
    static constexpr size_t MIN_LENGTH = 17;
    static constexpr uint8_t CRC_EXTRA = 175;
    static constexpr auto NAME = "AUCTION_BID_BUNDLE_RCV_PART";


    uint64_t agentID; /*< ID of agent generating the bundle */
    double bundleGenTime; /*< Bundle generation time */
    int8_t seqNum; /*< Last sequence number received (-1 to begin upload) */


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
        ss << "  seqNum: " << +seqNum << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << agentID;                       // offset: 0
        map << bundleGenTime;                 // offset: 8
        map << seqNum;                        // offset: 16
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> agentID;                       // offset: 0
        map >> bundleGenTime;                 // offset: 8
        map >> seqNum;                        // offset: 16
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
