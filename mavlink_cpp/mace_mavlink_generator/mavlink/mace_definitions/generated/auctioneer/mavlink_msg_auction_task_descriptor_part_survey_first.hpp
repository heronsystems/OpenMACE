// MESSAGE AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST support class

#pragma once

namespace mavlink {
namespace auctioneer {
namespace msg {

/**
 * @brief AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST message
 *
 * 
 */
struct AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST : mavlink::Message {
    static constexpr msgid_t MSG_ID = 10055;
    static constexpr size_t LENGTH = 43;
    static constexpr size_t MIN_LENGTH = 43;
    static constexpr uint8_t CRC_EXTRA = 54;
    static constexpr auto NAME = "AUCTION_TASK_DESCRIPTOR_PART_SURVEY_FIRST";


    uint64_t sentFrom; /*< ID of agent the task is being requested from */
    uint64_t creatorID; /*< Task creator ID */
    uint8_t taskID; /*< Creator local task ID */
    int8_t seqNum; /*< Sequence number */
    double sensorResolution; /*< Sensor resolution */
    double overlapHorizontal; /*< Horizontal overlap */
    double overlapVertical; /*< Vertical overlap */
    uint8_t coordinateType; /*< Coordinate type. See TaskDescriptor::TaskType */


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
        ss << "  sentFrom: " << sentFrom << std::endl;
        ss << "  creatorID: " << creatorID << std::endl;
        ss << "  taskID: " << +taskID << std::endl;
        ss << "  seqNum: " << +seqNum << std::endl;
        ss << "  sensorResolution: " << sensorResolution << std::endl;
        ss << "  overlapHorizontal: " << overlapHorizontal << std::endl;
        ss << "  overlapVertical: " << overlapVertical << std::endl;
        ss << "  coordinateType: " << +coordinateType << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << sentFrom;                      // offset: 0
        map << creatorID;                     // offset: 8
        map << sensorResolution;              // offset: 16
        map << overlapHorizontal;             // offset: 24
        map << overlapVertical;               // offset: 32
        map << taskID;                        // offset: 40
        map << seqNum;                        // offset: 41
        map << coordinateType;                // offset: 42
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> sentFrom;                      // offset: 0
        map >> creatorID;                     // offset: 8
        map >> sensorResolution;              // offset: 16
        map >> overlapHorizontal;             // offset: 24
        map >> overlapVertical;               // offset: 32
        map >> taskID;                        // offset: 40
        map >> seqNum;                        // offset: 41
        map >> coordinateType;                // offset: 42
    }
};

} // namespace msg
} // namespace auctioneer
} // namespace mavlink
