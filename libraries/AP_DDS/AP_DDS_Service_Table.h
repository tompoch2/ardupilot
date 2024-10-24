#include "uxr/client/client.h"

enum class ServiceIndex: uint8_t {
    ARMING_MOTORS,
    MODE_SWITCH,
    GET_RALLY,
    SET_RALLY
};

static inline constexpr uint8_t to_underlying(const ServiceIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}

constexpr struct AP_DDS_Client::Service_table AP_DDS_Client::services[] = {
    {
        .req_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .rep_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/arm_motorsService",
        .request_type = "ardupilot_msgs::srv::dds_::ArmMotors_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ArmMotors_Response_",
        .request_topic_name = "rq/ap/arm_motorsRequest",
        .reply_topic_name = "rr/ap/arm_motorsReply",
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
    {
        .req_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .rep_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/mode_switchService",
        .request_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Response_",
        .request_topic_name = "rq/ap/mode_switchRequest",
        .reply_topic_name = "rr/ap/mode_switchReply",
    },
    {
        .req_id = to_underlying(ServiceIndex::GET_RALLY),
        .rep_id = to_underlying(ServiceIndex::GET_RALLY),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/rally_getService",
        .request_type = "ardupilot_msgs::srv::dds_::RallyGet_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::RallyGet_Response_",
        .request_topic_name = "rq/ap/rally_getRequest",
        .reply_topic_name = "rr/ap/rally_getReply",
    },
    {
        .req_id = to_underlying(ServiceIndex::SET_RALLY),
        .rep_id = to_underlying(ServiceIndex::SET_RALLY),
        .service_rr = Service_rr::Replier,
        .service_name = "rs/ap/rally_setService",
        .request_type = "ardupilot_msgs::srv::dds_::RallySet_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::RallySet_Response_",
        .request_topic_name = "rq/ap/rally_setRequest",
        .reply_topic_name = "rr/ap/rally_setReply",
    },
};
