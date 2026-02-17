#include "composable_player/recorder_node.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace composable_player
{

RecorderNode::RecorderNode(const rclcpp::NodeOptions & options)
: Node("recorder", options),
  recording_(true)
{
  declare_parameter<std::string>("output_uri", "");
  declare_parameter<std::string>("storage_id", "mcap");
  declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
  declare_parameter<bool>("all_topics", false);
  declare_parameter<std::vector<std::string>>(
    "exclude_topics",
    std::vector<std::string>{"/rosout", "/parameter_events"});

  output_uri_ = get_parameter("output_uri").as_string();
  storage_id_ = get_parameter("storage_id").as_string();
  topics_ = get_parameter("topics").as_string_array();
  all_topics_ = get_parameter("all_topics").as_bool();
  exclude_topics_ = get_parameter("exclude_topics").as_string_array();

  if (output_uri_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'output_uri' is required");
    return;
  }

  if (topics_.empty() && !all_topics_) {
    RCLCPP_ERROR(get_logger(), "Specify 'topics' or set 'all_topics' to true");
    return;
  }

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_uri_;
  storage_options.storage_id = storage_id_;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options);

  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop",
    std::bind(&RecorderNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));

  discovery_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RecorderNode::discover_and_subscribe, this));

  discover_and_subscribe();

  RCLCPP_INFO(
    get_logger(), "RecorderNode initialized: uri=%s storage=%s all_topics=%s",
    output_uri_.c_str(), storage_id_.c_str(), all_topics_ ? "true" : "false");
}

RecorderNode::~RecorderNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  writer_.reset();
}

void RecorderNode::discover_and_subscribe()
{
  if (!recording_) {
    return;
  }

  auto topic_names_and_types = get_topic_names_and_types();
  std::unordered_set<std::string> exclude_set(exclude_topics_.begin(), exclude_topics_.end());
  std::unordered_set<std::string> target_set(topics_.begin(), topics_.end());

  for (const auto & [topic_name, type_list] : topic_names_and_types) {
    if (subscribed_topics_.count(topic_name) > 0) {
      continue;
    }

    if (exclude_set.count(topic_name) > 0) {
      continue;
    }

    if (!all_topics_ && target_set.find(topic_name) == target_set.end()) {
      continue;
    }

    if (type_list.empty()) {
      continue;
    }

    subscribe_to_topic(topic_name, type_list[0]);
  }
}

void RecorderNode::subscribe_to_topic(
  const std::string & topic_name, const std::string & topic_type)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!writer_) {
    return;
  }

  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";
  writer_->create_topic(topic_metadata);

  auto callback = [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      write_message(msg, topic_name);
    };

  auto subscription = create_generic_subscription(
    topic_name, topic_type, rclcpp::QoS(10), callback);

  subscriptions_.push_back(subscription);
  subscribed_topics_.insert(topic_name);

  RCLCPP_INFO(get_logger(), "Recording: %s [%s]", topic_name.c_str(), topic_type.c_str());
}

void RecorderNode::write_message(
  std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!recording_ || !writer_) {
    return;
  }

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic_name;
  bag_message->time_stamp = now().nanoseconds();

  const auto & rcl_msg = msg->get_rcl_serialized_message();

  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t,
    [](rcutils_uint8_array_t * p) {
      rcutils_uint8_array_fini(p);
      delete p;
    });

  *bag_message->serialized_data = rcutils_get_zero_initialized_uint8_array();
  auto ret = rcutils_uint8_array_init(
    bag_message->serialized_data.get(),
    rcl_msg.buffer_length,
    &rcutils_get_default_allocator());

  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to allocate serialized data for %s", topic_name.c_str());
    return;
  }

  std::memcpy(
    bag_message->serialized_data->buffer,
    rcl_msg.buffer,
    rcl_msg.buffer_length);
  bag_message->serialized_data->buffer_length = rcl_msg.buffer_length;

  writer_->write(bag_message);
}

void RecorderNode::on_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (recording_) {
    recording_ = false;
    discovery_timer_->cancel();
    subscriptions_.clear();
    writer_.reset();
    response->success = true;
    response->message = "Recording stopped";
    RCLCPP_INFO(get_logger(), "Recording stopped");
  } else {
    response->success = false;
    response->message = "Not recording";
  }
}

}  // namespace composable_player

RCLCPP_COMPONENTS_REGISTER_NODE(composable_player::RecorderNode)
